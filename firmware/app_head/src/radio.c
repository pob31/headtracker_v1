/*
 * radio.c — ESB uplink PTX with periodic PRX beacon-listen windows.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>

#include <esb.h>

#include "htk_protocol.h"
#include "htk_radio.h"
#include "radio.h"

LOG_MODULE_REGISTER(htk_radio, CONFIG_LOG_DEFAULT_LEVEL);

static K_MUTEX_DEFINE(radio_lock);
/* Beacons arrive at 10 Hz and a window is ~110 ms: 1-2 expected, 8 is slack
 * for multiple dongles in range. Drained by the control thread after each
 * listen window; overflow just drops (beacons repeat). */
K_MSGQ_DEFINE(beacon_q, sizeof(struct htk_beacon_rx), 8, 4);

static const uint8_t base_addr[4] = HTK_ADDR_BASE;
static bool in_ptx;

static void esb_handler(const struct esb_evt *event)
{
	struct esb_payload p;

	switch (event->evt_id) {
	case ESB_EVENT_TX_SUCCESS:
		break;
	case ESB_EVENT_TX_FAILED:
		/* Should not happen with noack payloads (no ACK to miss);
		 * flush so a wedged FIFO cannot stall the stream.
		 * VERIFY: TX_FAILED conditions for noack payloads in NCS ESB. */
		(void)esb_flush_tx();
		break;
	case ESB_EVENT_RX_RECEIVED:
		while (esb_read_rx_payload(&p) == 0) {
			if (p.length < sizeof(struct htk_beacon_hdr) ||
			    p.data[0] != HTK_PKT_BEACON) {
				continue; /* unknown air traffic: skip */
			}
			struct htk_beacon_rx b;

			b.len = MIN(p.length, sizeof(b.data));
			memcpy(b.data, p.data, b.len);
			(void)k_msgq_put(&beacon_q, &b, K_NO_WAIT);
		}
		break;
	}
}

/* ESB runs from HFXO; start it explicitly before esb_init(), as the NCS ESB
 * samples do (clock_control onoff service on the nRF52 clock driver). */
static int clocks_start(void)
{
	int err;
	int res;
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;

	clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	if (clk_mgr == NULL) {
		LOG_ERR("Unable to get HFCLK manager");
		return -ENXIO;
	}

	sys_notify_init_spinwait(&clk_cli.notify);

	err = onoff_request(clk_mgr, &clk_cli);
	if (err < 0) {
		LOG_ERR("HFCLK request failed: %d", err);
		return err;
	}

	do {
		err = sys_notify_fetch_result(&clk_cli.notify, &res);
		if (!err && res) {
			LOG_ERR("HFCLK could not be started: %d", res);
			return res;
		}
	} while (err);

	return 0;
}

/* (Re)initialize ESB in the given mode. PTX: uplink address, data out.
 * PRX: downlink address, beacons in. Caller holds radio_lock (or is init). */
static int esb_configure(bool ptx)
{
	int err;
	struct esb_config cfg = ESB_DEFAULT_CONFIG;
	uint8_t prefix[1] = { ptx ? HTK_PREFIX_UPLINK : HTK_PREFIX_DOWNLINK };

	cfg.protocol = ESB_PROTOCOL_ESB_DPL;
	cfg.mode = ptx ? ESB_MODE_PTX : ESB_MODE_PRX;
	cfg.event_handler = esb_handler;
	cfg.bitrate = ESB_BITRATE_2MBPS;
	cfg.crc = ESB_CRC_16BIT;
	/* Per-payload noack flag decides acking; all our traffic sets it. */
	cfg.selective_auto_ack = true;
	/* Default tx_mode ESB_TXMODE_AUTO: esb_write_payload() starts the
	 * transmission itself, no esb_start_tx() per packet. */

	err = esb_init(&cfg);
	if (err) {
		LOG_ERR("esb_init: %d", err);
		return err;
	}

	err = esb_set_base_address_0(base_addr);
	if (!err) {
		err = esb_set_prefixes(prefix, ARRAY_SIZE(prefix));
	}
	if (!err) {
		err = esb_set_rf_channel(HTK_RF_CHANNEL);
	}
	if (err) {
		LOG_ERR("esb address/channel config: %d", err);
		return err;
	}

	in_ptx = ptx;
	return 0;
}

int htk_radio_init(void)
{
	int err = clocks_start();

	if (err) {
		return err;
	}
	return esb_configure(true);
}

int htk_radio_send(const void *payload, uint8_t len)
{
	int err;

	if (len > CONFIG_ESB_MAX_PAYLOAD_LENGTH) {
		return -EMSGSIZE;
	}

	/* No-wait: during a listen window the sample is expendable. */
	if (k_mutex_lock(&radio_lock, K_NO_WAIT) != 0) {
		return -EBUSY;
	}

	if (!in_ptx) {
		k_mutex_unlock(&radio_lock);
		return -EAGAIN;
	}

	struct esb_payload p = {
		.pipe = 0,
		.noack = true,
		.length = len,
	};
	memcpy(p.data, payload, len);

	err = esb_write_payload(&p);
	if (err == -ENOMEM) {
		/* TX FIFO backed up (should not happen at 3% air time):
		 * drop the backlog, keep the freshest sample. */
		(void)esb_flush_tx();
		err = esb_write_payload(&p);
	}

	k_mutex_unlock(&radio_lock);
	return err;
}

void htk_radio_listen(uint32_t window_ms)
{
	int err;

	k_mutex_lock(&radio_lock, K_FOREVER);

	/* Let an in-flight packet leave the air (~180 us for 32 B at 2 Mbps)
	 * before tearing the radio down.
	 * VERIFY: whether esb_disable() aborts an active TX cleanly, or if
	 * ESB exposes a "TX idle" query worth polling here instead. */
	k_sleep(K_USEC(500));

	esb_disable();

	/* VERIFY: mode is fixed at esb_init(), so PTX<->PRX requires this
	 * full disable/init cycle (as of NCS 3.3); confirm no extra teardown
	 * (e.g. event re-registration) is needed on repeated cycles. */
	err = esb_configure(false);
	if (!err) {
		err = esb_start_rx();
		if (err) {
			LOG_ERR("esb_start_rx: %d", err);
		}
	}

	if (!err) {
		k_sleep(K_MSEC(window_ms));
		(void)esb_stop_rx();
	}

	esb_disable();
	err = esb_configure(true);
	if (err) {
		LOG_ERR("PTX restore failed: %d", err);
	}

	k_mutex_unlock(&radio_lock);
}

bool htk_radio_beacon_get(struct htk_beacon_rx *out)
{
	return k_msgq_get(&beacon_q, out, K_NO_WAIT) == 0;
}
