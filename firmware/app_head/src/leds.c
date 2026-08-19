/*
 * leds.c — LED patterns from a 50 ms k_timer tick.
 *
 * Pattern state lives entirely in the timer callback; the rest of the app
 * only flips atomics, so LED policy can't block or race the data path.
 * XIAO nRF52840 LEDs are active-low; the board DT flags encode that, so
 * logical 1 == on via gpio_pin_set_dt().
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/atomic.h>

#include "leds.h"

LOG_MODULE_REGISTER(htk_leds, CONFIG_LOG_DEFAULT_LEVEL);

static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

#define TICK_MS       50
#define CYCLE_TICKS   40  /* 2 s pattern period */
#define IDENTIFY_TICKS (3000 / TICK_MS)
#define TAP_TICKS     (200 / TICK_MS)

static atomic_t link_state = ATOMIC_INIT(LEDS_LINK_SEARCHING);
static atomic_t low_batt = ATOMIC_INIT(0);
static atomic_t identify_ticks = ATOMIC_INIT(0);
static atomic_t at_rest = ATOMIC_INIT(0);
static uint32_t tick;

static void set_rgb(int r, int g, int b)
{
	(void)gpio_pin_set_dt(&led_red, r);
	(void)gpio_pin_set_dt(&led_green, g);
	(void)gpio_pin_set_dt(&led_blue, b);
}

/* Runs in the system clock ISR; nRF GPIO set is ISR-safe and cheap. */
static void led_tick(struct k_timer *timer)
{
	ARG_UNUSED(timer);

	uint32_t t = tick % CYCLE_TICKS;

	tick++;

	if (atomic_get(&identify_ticks) > 0) {
		atomic_dec(&identify_ticks);
		set_rgb(1, 1, 1); /* white: IDENTIFY (or tap blip) */
		return;
	}

	if (atomic_get(&low_batt)) {
		/* red, short blink each cycle (steady red would out-draw
		 * the point of a low-battery warning) */
		set_rgb(t < 2, 0, 0);
		return;
	}

	if (atomic_get(&link_state) == LEDS_LINK_STREAMING) {
		/* green blink in motion; cyan while the drift gate holds yaw */
		set_rgb(0, t < 1, t < 1 && atomic_get(&at_rest));
	} else {
		int on = t < 5;              /* amber 250 ms every 2 s */
		set_rgb(on, on, 0);
	}
}

static K_TIMER_DEFINE(led_timer, led_tick, NULL);

int leds_init(void)
{
	const struct gpio_dt_spec *leds[] = { &led_red, &led_green, &led_blue };

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		if (!gpio_is_ready_dt(leds[i])) {
			LOG_ERR("LED GPIO not ready");
			return -ENODEV;
		}
		int err = gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);

		if (err) {
			return err;
		}
	}

	k_timer_start(&led_timer, K_MSEC(TICK_MS), K_MSEC(TICK_MS));
	return 0;
}

void leds_set_link(enum leds_link link)
{
	atomic_set(&link_state, link);
}

void leds_set_low_batt(bool low)
{
	atomic_set(&low_batt, low);
}

void leds_set_rest(bool rest)
{
	atomic_set(&at_rest, rest);
}

void leds_identify(void)
{
	atomic_set(&identify_ticks, IDENTIFY_TICKS);
}

void leds_tap(void)
{
	/* Shares the identify mechanism, shorter; an in-progress identify
	 * is not cut short by a tap. */
	if (atomic_get(&identify_ticks) < TAP_TICKS) {
		atomic_set(&identify_ticks, TAP_TICKS);
	}
}
