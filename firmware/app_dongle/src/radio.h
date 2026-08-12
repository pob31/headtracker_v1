/* ESB broadcast fabric, dongle side: PRX on the uplink address for tracker
 * data, periodic PTX beacons on the downlink address for presence + command
 * carriage (PROTOCOL.md Part 2).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_RADIO_H
#define APP_RADIO_H

#include <stddef.h>
#include <stdint.h>

/* Starts HF clock, ESB PRX on the uplink address, the RX processing thread's
 * feed, and the beacon timer. */
int radio_init(void);

/* Load a sensor-directed command payload (type + body, as received from the
 * host) into the pending beacon slot. It is embedded verbatim in the next
 * HTK_BEACON_CMD_REPEATS beacons, then cleared. Only the latest command is
 * kept (last-writer-wins, per PROTOCOL.md §2.3). Safe from any context. */
void radio_set_pending_cmd(const uint8_t *payload, size_t len);

#endif
