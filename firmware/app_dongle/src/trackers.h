/* Tracker table: per-id sequence/loss/rate accounting and the 1 Hz
 * STATUS + TRACKER_STAT emission (PROTOCOL.md §1.4, §2.5).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_TRACKERS_H
#define APP_TRACKERS_H

#include <stdint.h>

struct htk_tstatus;

#define TRACKERS_MAX 8

/* One ORIENT/RAW sample arrived from tracker `id` (radio thread). Updates
 * last-seen, per-second rate, and seq-gap loss accounting. */
void trackers_on_sample(uint16_t id, uint16_t seq);

/* A 1 Hz TSTATUS self-report arrived (radio thread). */
void trackers_on_tstatus(const struct htk_tstatus *ts);

/* Call exactly once per second: folds per-second counters into rates,
 * evicts trackers silent > 30 s, then emits STATUS + TRACKER_STATs. */
void trackers_tick_1hz(void);

/* Emit STATUS + one TRACKER_STAT per valid tracker now (GET_STATS path).
 * Uses the rates computed at the last 1 Hz tick; does not reset counters. */
void trackers_emit_stats(void);

#endif
