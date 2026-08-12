/* SPDX-License-Identifier: GPL-3.0-or-later */
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "leds.h"

LOG_MODULE_REGISTER(leds, CONFIG_LOG_DEFAULT_LEVEL);

/* xiao_ble aliases: led0 = red, led1 = green, led2 = blue (all active-low;
 * GPIO_ACTIVE_LOW comes from the board DT, so logical 1 = lit). */
static const struct gpio_dt_spec led_red   = GPIO_DT_SPEC_GET(DT_ALIAS(led0), gpios);
static const struct gpio_dt_spec led_green = GPIO_DT_SPEC_GET(DT_ALIAS(led1), gpios);
static const struct gpio_dt_spec led_blue  = GPIO_DT_SPEC_GET(DT_ALIAS(led2), gpios);

/* Green flicker: toggle every 16th packet (~13 Hz blink at 208 Hz traffic),
 * forced off 250 ms after traffic stops. */
#define ACTIVITY_DIV_MASK 0x0Fu
#define ACTIVITY_OFF_MS   250

static uint32_t activity_count;

static void activity_off_fn(struct k_work *work)
{
	ARG_UNUSED(work);
	gpio_pin_set_dt(&led_green, 0);
}
static K_WORK_DELAYABLE_DEFINE(activity_off_work, activity_off_fn);

int leds_init(void)
{
	const struct gpio_dt_spec *leds[] = { &led_red, &led_green, &led_blue };

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		if (!gpio_is_ready_dt(leds[i])) {
			return -ENODEV;
		}
		int err = gpio_pin_configure_dt(leds[i], GPIO_OUTPUT_INACTIVE);

		if (err) {
			return err;
		}
	}
	return 0;
}

void leds_set_blue(bool on)
{
	gpio_pin_set_dt(&led_blue, on);
}

void leds_set_red(bool on)
{
	gpio_pin_set_dt(&led_red, on);
}

void leds_activity(void)
{
	if ((++activity_count & ACTIVITY_DIV_MASK) == 0) {
		gpio_pin_toggle_dt(&led_green);
		k_work_reschedule(&activity_off_work, K_MSEC(ACTIVITY_OFF_MS));
	}
}
