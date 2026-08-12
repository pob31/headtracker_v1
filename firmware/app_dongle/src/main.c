/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * app_dongle: USB receiver dongle. Wiring only — the modules do the work:
 *   usb_io    CDC-ACM pipe, framing, host command dispatch
 *   radio     ESB PRX data path + presence beacons
 *   trackers  per-id stats table + 1 Hz STATUS/TRACKER_STAT emission
 *   sim       synthetic ORIENT stream (SIM_MODE)
 *
 * Streaming starts as soon as air packets arrive; no handshake or selection
 * is required (PROTOCOL.md §1.7). Beacons run whenever the dongle is
 * powered, which is what holds head units out of standby.
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <htk_protocol.h>

#include "leds.h"
#include "usb_io.h"
#include "radio.h"
#include "trackers.h"

LOG_MODULE_REGISTER(main, CONFIG_LOG_DEFAULT_LEVEL);

/* 1 Hz: rate fold-over, eviction, STATUS + TRACKER_STAT emission.
 * Runs on the system workqueue (frame encoding doesn't belong in a timer
 * ISR); GET_STATS reuses the same emission from the usb_io work context. */
static void stats_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	trackers_tick_1hz();
}
static K_WORK_DEFINE(stats_work, stats_work_fn);

static void stats_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&stats_work);
}
static K_TIMER_DEFINE(stats_timer, stats_timer_fn, NULL);

/* Blue heartbeat while USB is configured; dark otherwise. */
static void heartbeat_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	static bool on;

	on = usb_io_is_configured() && !on;
	leds_set_blue(on);
}
static K_TIMER_DEFINE(heartbeat_timer, heartbeat_timer_fn, NULL);

int main(void)
{
	int err = leds_init();

	if (err) {
		LOG_ERR("LED init failed (%d)", err);
	}

	err = usb_io_init();
	if (err) {
		LOG_ERR("USB init failed (%d)", err);
		leds_set_red(true);
	}

	err = radio_init();
	if (err) {
		LOG_ERR("radio init failed (%d)", err);
		leds_set_red(true);
	}

	k_timer_start(&stats_timer, K_SECONDS(1), K_SECONDS(1));
	k_timer_start(&heartbeat_timer, K_MSEC(500), K_MSEC(500));

	LOG_INF("headtracker dongle fw %u.%u.%u, proto %u",
		HTK_FW_MAJOR, HTK_FW_MINOR, HTK_FW_PATCH, HTK_PROTO_VERSION);
	return 0;
}
