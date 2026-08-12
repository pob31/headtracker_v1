/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * Simulator (F5): synthetic ORIENT stream as tracker HTK_ID_SIM at 208 Hz,
 * slow yaw sweep about world Z. Runs alongside real trackers; the SIM flag
 * and reserved id keep it unmistakable on the host side. Float math happens
 * in the system workqueue, not in the timer ISR.
 */
#include <zephyr/kernel.h>
#include <math.h>
#include <stdbool.h>

#include <htk_protocol.h>

#include "sim.h"
#include "usb_io.h"

#define SIM_RATE_HZ        208
#define SIM_PERIOD_US      (1000000 / SIM_RATE_HZ) /* ~4807 us */
#define SIM_YAW_RATE_DPS   10.0f
#define DEG_TO_RAD         0.017453293f
#define TWO_PI             6.283185307f

static bool active;
static uint16_t seq;      /* monotonic across enable/disable cycles */
static float yaw_rad;

static void sim_work_fn(struct k_work *work)
{
	ARG_UNUSED(work);

	yaw_rad += (SIM_YAW_RATE_DPS * DEG_TO_RAD) / (float)SIM_RATE_HZ;
	if (yaw_rad > TWO_PI) {
		yaw_rad -= TWO_PI;
	}

	/* Rotation about world Z (yaw): q = {cos(a/2), 0, 0, sin(a/2)}. */
	const struct htk_orient o = {
		.type = HTK_PKT_ORIENT,
		.id = HTK_ID_SIM,
		.seq = seq++,
		.t_us = (uint32_t)k_ticks_to_us_floor64(k_uptime_ticks()),
		.q_w = cosf(yaw_rad * 0.5f),
		.q_x = 0.0f,
		.q_y = 0.0f,
		.q_z = sinf(yaw_rad * 0.5f),
		.flags = HTK_ORIENT_SIM,
	};

	usb_io_send_payload(&o, sizeof(o));
}
static K_WORK_DEFINE(sim_work, sim_work_fn);

static void sim_timer_fn(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	k_work_submit(&sim_work);
}
static K_TIMER_DEFINE(sim_timer, sim_timer_fn, NULL);

void sim_set(bool on)
{
	if (on == active) {
		return;
	}
	active = on;
	if (on) {
		k_timer_start(&sim_timer, K_USEC(SIM_PERIOD_US),
			      K_USEC(SIM_PERIOD_US));
	} else {
		k_timer_stop(&sim_timer);
	}
}

bool sim_is_active(void)
{
	return active;
}
