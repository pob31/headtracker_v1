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

struct Vec3 {
    float x = 0.0f, y = 0.0f, z = 0.0f;
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

/* THE yaw metric for all reference bookkeeping: the world-Z twist angle of
 * q = heading(q) ⊗ tilt, i.e. 2*atan2(z, w) of heading(q), wrapped to
 * (-pi, pi]. NOT the same function as to_euler().yaw — the two disagree away
 * from level and Euler degenerates at pitch ±90°. Yaw re-anchoring is exact
 * in this metric and only this metric. */
float heading_angle(const Quat &q);

Quat yaw_quat(float rad);   /* pure world-Z rotation */
float wrap_pi(float rad);

/* World-down expressed in body coordinates: conj(q) ⊗ (0,0,-1) ⊗ q.
 * Invariant under ANY left-multiplied world yaw (drift, recenter,
 * auto-center, firmware yaw-hold) — the one signal yaw cannot poison. */
Vec3 gravity_body(const Quat &q);

/* Shortest-arc rotation carrying g to (0,0,-1). Its z-component is exactly 0
 * (it IS the swing factor of any rotation mapping g to down). Degenerate
 * input (g ≈ +Z: mounted upside-down) falls back to a 180° X rotation. */
Quat shortest_arc_to_down(const Vec3 &g);

EulerZYX to_euler(const Quat &q);

/* Per-tracker reference state. Typical flow (spec §1.6): boresight() once when
 * the sensor is strapped to a new rig (wearer level, looking at "front"), then
 * recenter() as often as wanted; the two compose internally. The Stabilizer
 * drives the same object continuously via set_tilt() (auto-level). */
class Recenterer {
public:
    /* Full-pose tare: absorbs arbitrary mounting orientation. Clears any
     * previous yaw recenter. */
    void boresight(const Quat &current);

    /* Yaw-only zero on top of the current boresight (routine drift fix). */
    void recenter(const Quat &current);

    /* Replace the tilt (right-side) reference and RE-ANCHOR the yaw so the
     * output heading does not jump: exact in the heading_angle() twist
     * metric (deliberately NOT in Euler yaw). `current` is the latest raw
     * sample. Returns δ = φ_new − φ_old, the shift the caller's own
     * yaw-derived state (e.g. an auto-center accumulator) must co-rotate by.
     * The new tilt should be a pure-swing quaternion (z == 0), as produced
     * by conj(shortest_arc_to_down(...)). */
    float set_tilt(const Quat &bore_inv_new, const Quat &current);

    Quat tilt() const { return bore_inv_; }

    /* Yaw reference as a scalar angle (twist metric), for estimators. */
    float yaw_angle() const;
    void set_yaw_angle(float rad);

    /* Reference-corrected orientation for rendering. */
    Quat apply(const Quat &raw) const;

    void reset(); /* back to raw pass-through */

private:
    Quat bore_inv_; /* identity until boresight()/set_tilt() */
    Quat yaw_inv_;  /* identity until recenter() */
};

} // namespace htk
