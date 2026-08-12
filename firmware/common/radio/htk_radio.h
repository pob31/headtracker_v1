/* Radio constants shared by head units and dongles.
 * THE thing to change if a venue's RF environment is hostile: channel (and,
 * for isolating separate installations, the address bytes).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_RADIO_H
#define HTK_RADIO_H

/* 2400 + channel MHz. 77 sits above Wi-Fi channel 13. */
#define HTK_RF_CHANNEL 77

/* Shared 4-byte base address + 1-byte prefixes (ESB pipe addressing).
 * Uplink: head units -> all receivers (data, no-ack broadcast).
 * Downlink: receiver beacons -> all head units (presence + commands, no-ack). */
#define HTK_ADDR_BASE      { 0xA9, 0x5E, 0x3C, 0xD7 }
#define HTK_PREFIX_UPLINK   0x48
#define HTK_PREFIX_DOWNLINK 0x21

#define HTK_ESB_MAX_PAYLOAD 32 /* dynamic payload length cap */

/* Timing (firmware-tuned, not protocol-normative; see PROTOCOL.md §2.3/2.4) */
#define HTK_BEACON_PERIOD_MS      100  /* dongle beacon interval */
#define HTK_BEACON_CMD_REPEATS    30   /* beacons carrying a pending command */
#define HTK_LISTEN_PERIOD_ACTIVE_MS  1000 /* head: beacon-listen interval, active */
#define HTK_LISTEN_PERIOD_STANDBY_MS 5000 /* head: beacon-listen interval, standby */
#define HTK_LISTEN_WINDOW_MS      110  /* head: listen window (> beacon period) */
#define HTK_STANDBY_AFTER_MS      15000 /* no beacon for this long -> standby */

#endif
