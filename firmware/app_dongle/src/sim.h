/* Simulator: 208 Hz synthetic ORIENT stream as tracker HTK_ID_SIM, so the
 * whole host path can be exercised with no head unit or radio link (F5).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef APP_SIM_H
#define APP_SIM_H

#include <stdbool.h>

void sim_set(bool on);
bool sim_is_active(void);

#endif
