/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * CDC-ACM host pipe.
 *
 * TX: complete encoded frames only, stored length-prefixed in a ring buffer;
 * the interrupt path drains ONE frame per tx-ready callback so no partial
 * frame is ever interleaved and every fill batch ends with the frame's 0x00
 * terminator (PRD risk 2: Windows CDC read latency — the host parser must
 * never wait across a 64-byte boundary for a frame end).
 *
 * RX: bytes accumulate and are split on 0x00 delimiters; each block is
 * htk_frame_decode()d and dispatched. Unknown types are ignored (forward
 * compatibility, PROTOCOL.md §1.3).
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>

#include <htk_protocol.h>
#include <htk_frame.h>

#include "usb_io.h"
#include "trackers.h"
#include "radio.h"
#include "sim.h"

LOG_MODULE_REGISTER(usb_io, CONFIG_LOG_DEFAULT_LEVEL);

static const struct device *const cdc_dev =
	DEVICE_DT_GET(DT_NODELABEL(cdc_acm_uart0));

static volatile bool usb_configured;

/* ---- TX: frame ring ----------------------------------------------------
 * Entries are [len:u8][frame bytes]; a frame is enqueued only if it fits
 * whole, else dropped. ~2 KB absorbs ~35 ORIENT frames (~170 ms of one
 * tracker at 208 Hz) across host scheduling hiccups. The lock spans the
 * two-part put/get so the consumer can never observe a length prefix
 * without its frame bytes. */
RING_BUF_DECLARE(tx_ring, 2048);
static struct k_spinlock tx_lock;
static uint32_t tx_dropped;

/* In-flight frame, consumer (irq callback) context only. */
static uint8_t tx_cur[HTK_MAX_FRAME];
static uint8_t tx_cur_len;
static uint8_t tx_cur_off;

/* ---- RX: byte ring (ISR producer) -> work (parser consumer) ------------ */
RING_BUF_DECLARE(rx_ring, 256);
static uint8_t rx_block[HTK_MAX_FRAME]; /* work context only */
static size_t  rx_block_len;
static bool    rx_block_overflow;
static uint32_t rx_bad_frames;

static void rx_work_fn(struct k_work *work);
static K_WORK_DEFINE(rx_work, rx_work_fn);

/* ---- host -> dongle dispatch (work context) ---------------------------- */

static void send_hello_resp(void)
{
	const struct htk_hello_resp resp = {
		.type = HTK_PKT_HELLO_RESP,
		.proto_ver = HTK_PROTO_VERSION,
		.fw_major = HTK_FW_MAJOR,
		.fw_minor = HTK_FW_MINOR,
		.fw_patch = HTK_FW_PATCH,
		.device = 1, /* dongle */
		.caps = HTK_CAP_QUAT | HTK_CAP_RAW | HTK_CAP_SIM | HTK_CAP_MULTI,
	};

	usb_io_send_payload(&resp, sizeof(resp));
}

static void dispatch_payload(const uint8_t *p, size_t len)
{
	switch (p[0]) {
	case HTK_PKT_HELLO:
		if (len == sizeof(struct htk_hello)) {
			/* Version check is the host's job (§1.7); always answer. */
			send_hello_resp();
		}
		break;

	case HTK_PKT_GET_STATS:
		if (len == sizeof(struct htk_get_stats)) {
			trackers_emit_stats();
		}
		break;

	case HTK_PKT_SIM_MODE:
		if (len == sizeof(struct htk_sim_mode)) {
			sim_set(p[1] != 0);
		}
		break;

	/* Sensor-directed commands: validate target id, then hand the payload
	 * verbatim to the beacon pending slot. Invalid ids are silently
	 * dropped ("rejected", §1.5). */
	case HTK_PKT_RESET_FUSION:
	case HTK_PKT_IDENTIFY:
	case HTK_PKT_SET_RATE:
	case HTK_PKT_SET_MODE: {
		size_t want = (p[0] == HTK_PKT_SET_RATE) ? sizeof(struct htk_set_rate)
			    : (p[0] == HTK_PKT_SET_MODE) ? sizeof(struct htk_set_mode)
			    : sizeof(struct htk_cmd_id);
		if (len != want) {
			break;
		}
		uint16_t id = sys_get_le16(&p[1]);

		if (id == HTK_ID_INVALID || id == HTK_ID_SIM) {
			break;
		}
		radio_set_pending_cmd(p, len);
		break;
	}

	default:
		/* Unknown type: skip without error (forward compatibility). */
		break;
	}
}

static void rx_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	uint8_t b;

	while (ring_buf_get(&rx_ring, &b, 1) == 1) {
		if (b != 0x00) {
			if (rx_block_len < sizeof(rx_block)) {
				rx_block[rx_block_len++] = b;
			} else {
				/* Oversized: discard until next delimiter. */
				rx_block_overflow = true;
			}
			continue;
		}
		if (rx_block_len > 0 && !rx_block_overflow) {
			uint8_t payload[HTK_MAX_PAYLOAD];
			size_t n = htk_frame_decode(rx_block, rx_block_len,
						    payload, sizeof(payload));
			if (n > 0) {
				dispatch_payload(payload, n);
			} else {
				rx_bad_frames++;
			}
		}
		rx_block_len = 0;
		rx_block_overflow = false;
	}
}

/* ---- UART interrupt path ----------------------------------------------- */

static void cdc_irq_handler(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	while (uart_irq_update(dev) && uart_irq_is_pending(dev)) {
		if (uart_irq_rx_ready(dev)) {
			uint8_t buf[64];
			int n = uart_fifo_read(dev, buf, sizeof(buf));

			if (n > 0) {
				/* Partial put on overflow is fine: the parser
				 * resynchronizes on the next 0x00. */
				ring_buf_put(&rx_ring, buf, n);
				k_work_submit(&rx_work);
			}
		}

		if (uart_irq_tx_ready(dev)) {
			if (tx_cur_off == tx_cur_len) {
				/* Fetch next frame ([len][bytes]) atomically. */
				k_spinlock_key_t key = k_spin_lock(&tx_lock);
				uint8_t len;

				if (ring_buf_get(&tx_ring, &len, 1) != 1) {
					k_spin_unlock(&tx_lock, key);
					uart_irq_tx_disable(dev);
					continue;
				}
				ring_buf_get(&tx_ring, tx_cur, len);
				k_spin_unlock(&tx_lock, key);
				tx_cur_len = len;
				tx_cur_off = 0;
			}

			int n = uart_fifo_fill(dev, &tx_cur[tx_cur_off],
					       tx_cur_len - tx_cur_off);
			if (n > 0) {
				tx_cur_off += n;
			}
			/* ONE frame per fill batch: once this frame is fully
			 * accepted (its 0x00 included), stop; the next
			 * tx-ready callback sends the next frame. If the CDC
			 * buffer stalled mid-frame, also stop — the remainder
			 * goes first on the next callback, so frames are never
			 * interleaved.
			 * VERIFY on hardware: with the legacy stack the CDC
			 * ring may still coalesce two small frames into one
			 * 64 B IN transfer; harmless (both are complete and
			 * 0x00-terminated, so the host parser never stalls),
			 * but confirm no partial-frame tail ever ends a
			 * transfer. The planned usbd migration gives per-
			 * transfer control. */
			break;
		}
	}
}

/* ---- API ---------------------------------------------------------------- */

void usb_io_send_payload(const void *payload, size_t len)
{
	uint8_t frame[HTK_MAX_FRAME];
	size_t flen = htk_frame_encode(payload, len, frame, sizeof(frame));

	if (flen == 0) {
		return;
	}
	if (!usb_configured) {
		/* No host: don't age frames in the ring. */
		return;
	}

	k_spinlock_key_t key = k_spin_lock(&tx_lock);

	if (ring_buf_space_get(&tx_ring) >= flen + 1) {
		uint8_t l = (uint8_t)flen;

		ring_buf_put(&tx_ring, &l, 1);
		ring_buf_put(&tx_ring, frame, flen);
		k_spin_unlock(&tx_lock, key);
		uart_irq_tx_enable(cdc_dev);
	} else {
		/* Never block the radio path on a slow host: drop + count. */
		tx_dropped++;
		k_spin_unlock(&tx_lock, key);
		if ((tx_dropped & 0x3FFu) == 1) {
			LOG_WRN("TX ring full, %u frames dropped", tx_dropped);
		}
	}
}

uint32_t usb_io_tx_dropped(void)
{
	return tx_dropped;
}

bool usb_io_is_configured(void)
{
	return usb_configured;
}

static void usb_status_cb(enum usb_dc_status_code status, const uint8_t *param)
{
	ARG_UNUSED(param);

	switch (status) {
	case USB_DC_CONFIGURED:
		usb_configured = true;
		break;
	case USB_DC_DISCONNECTED:
	case USB_DC_SUSPEND:
	case USB_DC_RESET:
		usb_configured = false;
		break;
	default:
		break;
	}
}

int usb_io_init(void)
{
	if (!device_is_ready(cdc_dev)) {
		return -ENODEV;
	}

	/* Status callback instead of usb_enable(NULL): the configured flag
	 * gates TX and drives the blue heartbeat. */
	int err = usb_enable(usb_status_cb);

	if (err) {
		return err;
	}

	uart_irq_callback_set(cdc_dev, cdc_irq_handler);
	uart_irq_rx_enable(cdc_dev);
	return 0;
}
