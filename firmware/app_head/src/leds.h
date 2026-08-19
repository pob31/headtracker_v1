/*
 * leds.h — head unit LED state language (PRD F11).
 *
 * green short blink  = streaming (receiver heard), in motion
 * cyan short blink   = streaming AND at rest (drift gate armed, yaw held)
 * amber slow blink   = no receiver heard (searching/standby)
 * red                = battery low (stub until battery ADC lands)
 * white solid 3 s    = IDENTIFY command
 * white blip 200 ms  = double-tap acknowledged
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef APP_HEAD_LEDS_H
#define APP_HEAD_LEDS_H

#include <stdbool.h>

enum leds_link {
	LEDS_LINK_STREAMING,  /* ACTIVE: receiver beacon heard recently */
	LEDS_LINK_SEARCHING,  /* STANDBY / no receiver heard */
};

int leds_init(void);
void leds_set_link(enum leds_link link);
void leds_set_rest(bool at_rest); /* streaming blink green->cyan while held */
void leds_set_low_batt(bool low);  /* TODO: drive from battery ADC */
void leds_identify(void);          /* white for 3 s, then back to state */
void leds_tap(void);               /* white blip 200 ms: tap acknowledged */

#endif /* APP_HEAD_LEDS_H */
