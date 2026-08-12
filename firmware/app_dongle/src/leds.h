/* LED policy: blue = USB-configured heartbeat, green = air RX activity,
 * red = fatal init error. Board LEDs are active-low; the polarity lives in
 * the board devicetree, so all levels here are logical.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_LEDS_H
#define APP_LEDS_H

#include <stdbool.h>

int  leds_init(void);
void leds_set_blue(bool on);
void leds_set_red(bool on);

/* Call once per received air packet (thread context). Flickers green while
 * traffic flows; goes dark shortly after it stops. */
void leds_activity(void);

#endif
