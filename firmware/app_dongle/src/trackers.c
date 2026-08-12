/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Tracker table (PROTOCOL.md §1.4/§2.5): fixed 8 slots keyed by tracker id.
 * Loss/rate numbers are as observed by THIS dongle — every receiver keeps
 * its own. Called from the radio processing thread (updates) and from the
 * system workqueue (1 Hz tick, GET_STATS); a spinlock guards the table and
 * frame encoding happens outside it on a snapshot.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include <htk_protocol.h>

#include "trackers.h"
#include "usb_io.h"
#include "sim.h"

LOG_MODULE_REGISTER(trackers, CONFIG_LOG_DEFAULT_LEVEL);

#define LIVE_WINDOW_MS   3000   /* STATUS.n_trackers: seen within 3 s */
#define LINK_UP_MS       500    /* TRACKER_STAT LINK_UP flag */
#define EVICT_SILENT_MS  30000  /* drop from table after 30 s silence */

struct tracker {
	bool     valid;
	bool     have_seq;      /* last_seq meaningful (first packet seen) */
	uint16_t id;
	int64_t  last_seen;     /* k_uptime_get() at last packet */
	uint16_t last_seq;
	uint32_t seq_lost;      /* cumulative since dongle boot */
	uint16_t pkt_count;     /* ORIENT/RAW in the current second */
	uint16_t rate;          /* previous second's pkt_count */
	uint16_t vbat_mV;       /* self-reported; 0 = unknown */
	uint8_t  fw[3];         /* self-reported */
	uint8_t  mode;          /* self-reported HTK_MODE_* */
};

static struct tracker table[TRACKERS_MAX];
static struct k_spinlock lock;

/* All air packets (ORIENT + RAW + TSTATUS) for STATUS.rx_rate. */
static uint32_t rx_count_this_second;
static uint16_t rx_rate;

/* Must be called under `lock`. Returns NULL when the table is full of
 * recently-active trackers (the stream is still forwarded to USB; the
 * tracker just goes unaccounted until a slot frees — beyond-spec load). */
static struct tracker *claim(uint16_t id, int64_t now)
{
	struct tracker *free_slot = NULL;
	struct tracker *stale = NULL;

	for (int i = 0; i < TRACKERS_MAX; i++) {
		struct tracker *t = &table[i];

		if (t->valid && t->id == id) {
			return t;
		}
		if (!t->valid) {
			if (free_slot == NULL) {
				free_slot = t;
			}
		} else if ((now - t->last_seen) > EVICT_SILENT_MS &&
			   stale == NULL) {
			stale = t;
		}
	}

	struct tracker *slot = (free_slot != NULL) ? free_slot : stale;

	if (slot != NULL) {
		memset(slot, 0, sizeof(*slot));
		slot->valid = true;
		slot->id = id;
	}
	return slot;
}

void trackers_on_sample(uint16_t id, uint16_t seq)
{
	int64_t now = k_uptime_get();

	K_SPINLOCK(&lock) {
		rx_count_this_second++;

		struct tracker *t = claim(id, now);

		if (t == NULL) {
			K_SPINLOCK_BREAK;
		}
		t->last_seen = now;
		if (t->pkt_count < UINT16_MAX) {
			t->pkt_count++;
		}

		if (!t->have_seq) {
			t->have_seq = true;
			t->last_seq = seq;
		} else {
			/* Gap accounting modulo 2^16 (§2.5). step==1 is the
			 * normal case; a gap of n adds n-1. No retransmits
			 * exist, so step==0 (duplicate) or step>0x8000
			 * (stale/reordered) should not happen — guard anyway
			 * and leave last_seq at the newer value. */
			uint16_t step = (uint16_t)(seq - t->last_seq);

			if (step != 0 && step <= 0x8000u) {
				t->seq_lost += step - 1u;
				t->last_seq = seq;
			}
		}
	}
}

void trackers_on_tstatus(const struct htk_tstatus *ts)
{
	int64_t now = k_uptime_get();

	K_SPINLOCK(&lock) {
		rx_count_this_second++;

		struct tracker *t = claim(ts->id, now);

		if (t == NULL) {
			K_SPINLOCK_BREAK;
		}
		t->last_seen = now;
		t->vbat_mV = ts->vbat_mV;
		t->fw[0] = ts->fw_major;
		t->fw[1] = ts->fw_minor;
		t->fw[2] = ts->fw_patch;
		t->mode = ts->mode;
	}
}

/* Snapshot + emit, encoding outside the lock. */
static void emit_locked_snapshot(void)
{
	struct tracker snap[TRACKERS_MAX];
	uint16_t rate_snap;
	int64_t now = k_uptime_get();

	K_SPINLOCK(&lock) {
		memcpy(snap, table, sizeof(snap));
		rate_snap = rx_rate;
	}

	uint8_t n_live = 0;

	for (int i = 0; i < TRACKERS_MAX; i++) {
		if (snap[i].valid && (now - snap[i].last_seen) < LIVE_WINDOW_MS) {
			n_live++;
		}
	}

	const struct htk_status status = {
		.type = HTK_PKT_STATUS,
		.uptime_ms = (uint32_t)k_uptime_get(),
		.rx_rate = rate_snap,
		.n_trackers = n_live,
		.flags = sim_is_active() ? HTK_STATUS_SIM_ACTIVE : 0,
	};
	usb_io_send_payload(&status, sizeof(status));

	for (int i = 0; i < TRACKERS_MAX; i++) {
		const struct tracker *t = &snap[i];

		if (!t->valid) {
			continue;
		}
		int64_t age = now - t->last_seen;
		const struct htk_tracker_stat stat = {
			.type = HTK_PKT_TRACKER_STAT,
			.id = t->id,
			.age_ms = (age < 0) ? 0 : (uint32_t)age,
			.rate = t->rate,
			.seq_lost = t->seq_lost,
			.vbat_mV = t->vbat_mV,
			.fw_major = t->fw[0],
			.fw_minor = t->fw[1],
			.fw_patch = t->fw[2],
			.mode = t->mode,
			.flags = (age < LINK_UP_MS) ? HTK_TSTAT_LINK_UP : 0,
		};
		usb_io_send_payload(&stat, sizeof(stat));
	}
}

void trackers_tick_1hz(void)
{
	int64_t now = k_uptime_get();

	K_SPINLOCK(&lock) {
		rx_rate = (rx_count_this_second > UINT16_MAX)
			  ? UINT16_MAX : (uint16_t)rx_count_this_second;
		rx_count_this_second = 0;

		for (int i = 0; i < TRACKERS_MAX; i++) {
			struct tracker *t = &table[i];

			if (!t->valid) {
				continue;
			}
			t->rate = t->pkt_count;
			t->pkt_count = 0;
			if ((now - t->last_seen) > EVICT_SILENT_MS) {
				t->valid = false;
			}
		}
	}

	emit_locked_snapshot();
}

void trackers_emit_stats(void)
{
	emit_locked_snapshot();
}
