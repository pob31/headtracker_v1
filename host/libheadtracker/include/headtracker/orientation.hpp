/* Client-side orientation math per PROTOCOL.md §1.6 — recentering and
 * boresight are per-listener state and deliberately NOT wire commands.
 *
 * Conventions (spec §1.3): Hamilton quaternion w,x,y,z, unit norm, rotation
 * body->world; body X forward, Y left, Z up; world Z up. Yaw about world Z.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include "htk_protocol.h"

namespace htk {

struct Quat {
    float w = 1.0f, x = 0.0f, y = 0.0f, z = 0.0f;
};

struct EulerZYX {
    float yaw;   /* rad, about world Z */
    float pitch; /* rad, about intermediate Y' */
    float roll;  /* rad, about body X'' */
};

Quat quat_of(const htk_orient &o); /* extract + normalize */
Quat multiply(const Quat &a, const Quat &b);
Quat conjugate(const Quat &q);
Quat normalized(const Quat &q);

/* Yaw-only component (w,0,0,z normalized); identity when degenerate
 * (gimbal-vertical poses where w and z both vanish). */
Quat heading(const Quat &q);

EulerZYX to_euler(const Quat &q);

/* Per-tracker reference state. Typical flow (spec §1.6): boresight() once when
 * the sensor is strapped to a new rig (wearer level, looking at "front"), then
 * recenter() as often as wanted; the two compose internally. */
class Recenterer {
public:
    /* Full-pose tare: absorbs arbitrary mounting orientation. Clears any
     * previous yaw recenter. */
    void boresight(const Quat &current);

    /* Yaw-only zero on top of the current boresight (routine drift fix). */
    void recenter(const Quat &current);

    /* Reference-corrected orientation for rendering. */
    Quat apply(const Quat &raw) const;

    void reset(); /* back to raw pass-through */

private:
    Quat bore_inv_; /* identity until boresight() */
    Quat yaw_inv_;  /* identity until recenter() */
};

} // namespace htk
