/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * ESB broadcast fabric, dongle side (PROTOCOL.md Part 2).
 *
 * Normal state: PRX on the uplink address (pipe 0), receiving no-ack data
 * packets from every head unit in range. The ESB IRQ handler only copies
 * payloads into a message queue; a processing thread validates them, feeds
 * the tracker table, and forwards ORIENT/RAW verbatim to the USB pipe (air
 * payload bytes == USB payload bytes, §2.2).
 *
 * Every HTK_BEACON_PERIOD_MS the radio briefly flips to PTX on the downlink
 * address to emit one no-ack presence beacon, optionally carrying the pending
 * sensor-directed command, then returns to PRX. RX is deaf during that
 * window; at 2 Mbps the beacon itself is ~100 us on air, so the loss shows
 * up as a fraction of a percent in seq_lost at worst.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <esb.h>
#include <string.h>

#include <htk_protocol.h>
#include <htk_radio.h>

#include "radio.h"
#include "trackers.h"
#include "usb_io.h"
#include "leds.h"

LOG_MODULE_REGISTER(radio, CONFIG_LOG_DEFAULT_LEVEL);

/* Largest embedded command payload is htk_set_rate (5 B). */
#define PENDING_CMD_MAX 8

static struct {
	uint8_t buf[PENDING_CMD_MAX];
	uint8_t len;
	uint8_t repeats_left; /* beacons still carrying this command */
} pending_cmd;
static struct k_spinlock pending_lock;

static uint8_t beacon_seq;
static uint32_t rx_queue_overruns;

static K_SEM_DEFINE(tx_done_sem, 0, 1);

/* ESB IRQ -> processing thread hand-off. 64 entries ~= 300 ms of one
 * tracker at 208 Hz; overruns are counted, not blocked on. */
struct radio_pkt {
	uint8_t len;
	uint8_t data[HTK_ESB_MAX_PAYLOAD];
};
K_MSGQ_DEFINE(rx_msgq, sizeof(struct radio_pkt), 64, 1);

/* Beacons run on a dedicated workqueue so a busy system workqueue can never
 * stretch the beacon period (head units time their listen windows on it). */
static struct k_work_q radio_wq;
static K_THREAD_STACK_DEFINE(radio_wq_stack, 1024);
#define RADIO_WQ_PRIO 2

/* ---- ESB event handler (radio IRQ context: copy out, nothing else) ------ */

static void esb_event_handler(const struct esb_evt *event)
{
	switch (event->evt_id) {
	case ESB_EVENT_RX_RECEIVED: {
		struct esb_payload payload;

		while (esb_read_rx_payload(&payload) == 0) {
			struct radio_pkt pkt;

			pkt.len = payload.length;
			memcpy(pkt.data, payload.data, payload.length);
			if (k_msgq_put(&rx_msgq, &pkt, K_NO_WAIT) != 0) {
				rx_queue_overruns++;
			}
		}
		break;
	}
	case ESB_EVENT_TX_SUCCESS:
	case ESB_EVENT_TX_FAILED:
		/* no-ack TX never reports FAILED, but treat both as "done". */
		k_sem_give(&tx_done_sem);
		break;
	default:
		break;
	}
}

/* ---- mode (re)configuration --------------------------------------------- */

static int esb_setup(enum esb_mode mode)
{
	static const uint8_t base_addr[4] = HTK_ADDR_BASE;
	uint8_t prefix = (mode == ESB_MODE_PTX) ? HTK_PREFIX_DOWNLINK
						: HTK_PREFIX_UPLINK;
	struct esb_config config = ESB_DEFAULT_CONFIG;
	int err;

	config.protocol = ESB_PROTOCOL_ESB_DPL;
	config.mode = mode;
	config.event_handler = esb_event_handler;
	config.bitrate = ESB_BITRATE_2MBPS;
	config.crc = ESB_CRC_16BIT;
	/* Honor per-packet noack in both directions; nothing on this fabric
	 * is ever ACKed (§2.1). Retransmit settings are moot under noack. */
	config.selective_auto_ack = true;

	err = esb_init(&config);
	if (err) {
		return err;
	}
	err = esb_set_base_address_0(base_addr);
	if (err) {
		return err;
	}
	err = esb_set_prefixes(&prefix, 1);
	if (err) {
		return err;
	}
	/* Only pipe 0 in use. VERIFY: NCS 3.3 esb_set_prefixes(len=1) leaves
	 * other pipes disabled; otherwise add esb_enable_pipes(BIT(0)). */
	return esb_set_rf_channel(HTK_RF_CHANNEL);
}

/* ---- beacon TX (radio workqueue) ---------------------------------------- */

static void beacon_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	struct esb_payload tx;
	struct htk_beacon_hdr hdr = {
		.type = HTK_PKT_BEACON,
		.beacon_seq = beacon_seq++,
	};

	memset(&tx, 0, sizeof(tx));
	tx.pipe = 0;
	tx.noack = true;
	memcpy(tx.data, &hdr, sizeof(hdr));
	tx.length = sizeof(hdr);

	K_SPINLOCK(&pending_lock) {
		if (pending_cmd.repeats_left > 0) {
			memcpy(&tx.data[tx.length], pending_cmd.buf,
			       pending_cmd.len);
			tx.length += pending_cmd.len;
			pending_cmd.repeats_left--;
		}
	}

	/* PRX -> PTX -> PRX. RX is deaf for the whole switchover.
	 * VERIFY on hardware: measure the total gap (esb_disable + esb_init
	 * twice + ~100 us on air); expected well under 1 ms, i.e. <1% of the
	 * beacon period. If re-init proves too slow, cache the two configs or
	 * use radio shorts — measure first. */
	esb_stop_rx();
	esb_disable();

	if (esb_setup(ESB_MODE_PTX) == 0) {
		k_sem_reset(&tx_done_sem);
		/* ESB_TXMODE_AUTO (default config): writing the payload
		 * starts transmission; esb_start_tx() is only for manual
		 * mode. VERIFY against NCS 3.3 esb_ptx sample. */
		if (esb_write_payload(&tx) == 0) {
			if (k_sem_take(&tx_done_sem, K_MSEC(10)) != 0) {
				LOG_WRN("beacon TX completion timeout");
			}
		} else {
			LOG_WRN("beacon write failed");
		}
		esb_disable();
	}

	int err = esb_setup(ESB_MODE_PRX);

	if (err == 0) {
		err = esb_start_rx();
	}
	if (err) {
		LOG_ERR("PRX restore failed (%d)", err);
		leds_set_red(true);
	}
}
static K_WORK_DEFINE(beacon_work, beacon_work_fn);

static void beacon_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit_to_queue(&radio_wq, &beacon_work);
}
static K_TIMER_DEFINE(beacon_timer, beacon_timer_fn, NULL);

/* ---- RX processing thread ----------------------------------------------- */

static void rx_thread_fn(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	struct radio_pkt pkt;

	for (;;) {
		k_msgq_get(&rx_msgq, &pkt, K_FOREVER);
		if (pkt.len < 1) {
			continue;
		}

		switch (pkt.data[0]) {
		case HTK_PKT_ORIENT:
			if (pkt.len != sizeof(struct htk_orient)) {
				break;
			}
			{
				const struct htk_orient *o =
					(const struct htk_orient *)pkt.data;
				trackers_on_sample(o->id, o->seq);
				/* Air payload == USB payload (§2.2): forward
				 * the bytes verbatim; usb_io adds CRC + COBS. */
				usb_io_send_payload(pkt.data, pkt.len);
				leds_activity();
			}
			break;

		case HTK_PKT_RAW:
			if (pkt.len != sizeof(struct htk_raw)) {
				break;
			}
			{
				const struct htk_raw *r =
					(const struct htk_raw *)pkt.data;
				trackers_on_sample(r->id, r->seq);
				usb_io_send_payload(pkt.data, pkt.len);
				leds_activity();
			}
			break;

		case HTK_PKT_TSTATUS:
			if (pkt.len != sizeof(struct htk_tstatus)) {
				break;
			}
			/* Air-only: merged into TRACKER_STAT bookkeeping,
			 * never forwarded to USB. */
			trackers_on_tstatus(
				(const struct htk_tstatus *)pkt.data);
			leds_activity();
			break;

		default:
			/* Unknown air type: ignore (forward compatibility). */
			break;
		}
	}
}
K_THREAD_DEFINE(radio_rx_tid, 1536, rx_thread_fn, NULL, NULL, NULL,
		K_PRIO_PREEMPT(5), 0, 0);

/* ---- init ---------------------------------------------------------------- */

/* HF crystal must run before esb_init (NCS esb samples' clocks_start()).
 * VERIFY: this is the NCS 3.3 / Zephyr 4.1 onoff pattern for nRF52; newer
 * SoCs use a different clock-control API. */
static int clocks_start(void)
{
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	int err;
	int res;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (clk_mgr == NULL) {
		return -ENODEV;
	}

	sys_notify_init_spinwait(&clk_cli.notify);
	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
	} while (err == -EAGAIN);

	if (err < 0) {
		return err;
	}
	return res;
}

void radio_set_pending_cmd(const uint8_t *payload, size_t len)
{
	if (len == 0 || len > PENDING_CMD_MAX ||
	    len > HTK_ESB_MAX_PAYLOAD - sizeof(struct htk_beacon_hdr)) {
		return;
	}

	K_SPINLOCK(&pending_lock) {
		/* Latest command wins (§2.3); repeats restart. */
		memcpy(pending_cmd.buf, payload, len);
		pending_cmd.len = (uint8_t)len;
		pending_cmd.repeats_left = HTK_BEACON_CMD_REPEATS;
	}
}

int radio_init(void)
{
	int err = clocks_start();

	if (err) {
		return err;
	}

	err = esb_setup(ESB_MODE_PRX);
	if (err) {
		return err;
	}
	err = esb_start_rx();
	if (err) {
		return err;
	}

	k_work_queue_start(&radio_wq, radio_wq_stack,
			   K_THREAD_STACK_SIZEOF(radio_wq_stack),
			   RADIO_WQ_PRIO, NULL);
	k_thread_name_set(&radio_wq.thread, "radio_wq");

	k_timer_start(&beacon_timer, K_MSEC(HTK_BEACON_PERIOD_MS),
		      K_MSEC(HTK_BEACON_PERIOD_MS));

	LOG_INF("ESB PRX up: ch %d, beacons every %d ms",
		HTK_RF_CHANNEL, HTK_BEACON_PERIOD_MS);
	return 0;
}
