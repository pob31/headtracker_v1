/*
 * radio.h — head unit ESB radio: no-ack uplink PTX + periodic beacon listen.
 *
 * Ownership model: the control thread (main) owns radio *reconfiguration*
 * (PTX <-> PRX for beacon-listen windows) and holds the radio lock for the
 * whole window; the fusion thread calls htk_radio_send() with a no-wait
 * lock and simply drops the sample if a listen is in progress — a lost
 * sample is superseded ~5 ms later by design (PROTOCOL.md §2.1).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef APP_HEAD_RADIO_H
#define APP_HEAD_RADIO_H

#include <stdbool.h>
#include <stdint.h>

#include "htk_radio.h"

/* One received beacon payload (htk_beacon_hdr + optional embedded command). */
struct htk_beacon_rx {
	uint8_t len;
	uint8_t data[HTK_ESB_MAX_PAYLOAD];
};

/* Start HFCLK, bring ESB up in PTX on the uplink address. */
int htk_radio_init(void);

/* Queue one no-ack payload on the uplink pipe. Returns -EBUSY (listen in
 * progress) or -EAGAIN without transmitting; both mean "drop and move on". */
int htk_radio_send(const void *payload, uint8_t len);

/* Blocking: switch to PRX on the downlink address for window_ms, collecting
 * beacon payloads, then restore PTX. Call from the control thread only. */
void htk_radio_listen(uint32_t window_ms);

/* Dequeue one beacon collected during the last listen window.
 * Returns false when the queue is empty. */
bool htk_radio_beacon_get(struct htk_beacon_rx *out);

/* Bring-up diagnostics (cumulative). */
struct htk_radio_debug {
	uint32_t listens;        /* listen windows executed */
	uint32_t cfg_fail;       /* PRX reconfigure/start failures */
	uint32_t rx_events;      /* payloads seen by the RX handler (any type) */
	uint32_t beacons_queued; /* beacons handed to the control thread */
};
void htk_radio_get_debug(struct htk_radio_debug *out);

#endif /* APP_HEAD_RADIO_H */
