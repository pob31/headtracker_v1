/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "headtracker/orientation.hpp"

#include <cmath>

namespace htk {

Quat multiply(const Quat &a, const Quat &b)
{
    return {
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
    };
}

Quat conjugate(const Quat &q)
{
    return { q.w, -q.x, -q.y, -q.z };
}

Quat normalized(const Quat &q)
{
    const float n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
    /* A frame can pass CRC and still carry a non-finite float: the CRC covers
     * the bytes, not their meaning. Both halves of this guard are load-bearing:
     *   - positive logic, because `n2 < 1e-12f` is FALSE when n2 is NaN and
     *     would let 1/sqrt(NaN) straight through;
     *   - isfinite, because an infinite component makes n2 infinite, which
     *     PASSES a lower-bound test and then yields inf * (1/sqrt(inf)) =
     *     inf * 0 = NaN.
     * Non-finite in any component makes n2 non-finite, so this one test covers
     * every way the wire can poison the result. */
    if (!(std::isfinite(n2) && n2 > 1e-12f)) {
        return {}; /* degenerate wire data -> identity, never NaN */
    }
    const float inv = 1.0f / std::sqrt(n2);
    return { q.w * inv, q.x * inv, q.y * inv, q.z * inv };
}

Quat quat_of(const htk_orient &o)
{
    return normalized({ o.q_w, o.q_x, o.q_y, o.q_z });
}

Quat heading(const Quat &q)
{
    const float n2 = q.w * q.w + q.z * q.z;
    if (!(std::isfinite(n2) && n2 > 1e-12f)) { /* NaN/inf-safe, see normalized() */
        return {};
    }
    const float inv = 1.0f / std::sqrt(n2);
    return { q.w * inv, 0.0f, 0.0f, q.z * inv };
}

float wrap_pi(float rad)
{
    constexpr float kPi = 3.14159265358979f;

    if (!std::isfinite(rad)) {
        return 0.0f;
    }
    while (rad > kPi) {
        rad -= 2.0f * kPi;
    }
    while (rad < -kPi) {
        rad += 2.0f * kPi;
    }
    return rad;
}

float heading_angle(const Quat &q)
{
    const float n2 = q.w * q.w + q.z * q.z;
    if (!(std::isfinite(n2) && n2 > 1e-12f)) {
        return 0.0f; /* gimbal-vertical or poisoned: heading undefined */
    }
    return wrap_pi(2.0f * std::atan2(q.z, q.w));
}

Quat yaw_quat(float rad)
{
    return { std::cos(rad * 0.5f), 0.0f, 0.0f, std::sin(rad * 0.5f) };
}

Vec3 gravity_body(const Quat &q)
{
    /* R(q)^T * (0,0,-1) = negated third row of R(q). Callers pass normalized
     * quaternions (quat_of/normalized both sanitize), so no guard here. */
    return {
        -(2.0f * (q.x * q.z - q.w * q.y)),
        -(2.0f * (q.y * q.z + q.w * q.x)),
        -(1.0f - 2.0f * (q.x * q.x + q.y * q.y)),
    };
}

Quat shortest_arc_to_down(const Vec3 &g)
{
    const float n2 = g.x * g.x + g.y * g.y + g.z * g.z;
    if (!(std::isfinite(n2) && n2 > 1e-12f)) {
        return {}; /* no usable gravity direction: identity */
    }
    const float inv = 1.0f / std::sqrt(n2);
    const float ux = g.x * inv, uy = g.y * inv, uz = g.z * inv;

    /* Half-angle construction of the rotation u -> (0,0,-1):
     * w = 1 + u·(0,0,-1) = 1 - uz ; axis = u × (0,0,-1) = (-uy, ux, 0).
     * The z-component is exactly zero: this IS the swing factor. */
    if (1.0f - uz < 1e-6f) {
        /* u ≈ +Z (mounted upside-down): 180° about X */
        return { 0.0f, 1.0f, 0.0f, 0.0f };
    }
    return normalized({ 1.0f - uz, -uy, ux, 0.0f });
}

EulerZYX to_euler(const Quat &q)
{
    EulerZYX e;
    e.yaw = std::atan2(2.0f * (q.w * q.z + q.x * q.y),
                       1.0f - 2.0f * (q.y * q.y + q.z * q.z));
    float sp = 2.0f * (q.w * q.y - q.x * q.z);
    if (sp > 1.0f) {
        sp = 1.0f;
    } else if (sp < -1.0f) {
        sp = -1.0f;
    }
    e.pitch = std::asin(sp);
    e.roll = std::atan2(2.0f * (q.w * q.x + q.y * q.z),
                        1.0f - 2.0f * (q.x * q.x + q.y * q.y));
    return e;
}

void Recenterer::boresight(const Quat &current)
{
    /* Two facts shape this:
     * 1. RIGHT-side inverse: the mounting offset composes on the body side
     *    of a body->world quaternion (q_sensor = q_head ⊗ q_mount), so the
     *    tare divides on the right; q ⊗ q_ref⁻¹ is the rotation since
     *    capture, expressed in world coordinates.
     * 2. Heading conjugation: the fusion's world X/Y horizontal axes are
     *    arbitrary (6DoF yaw origin), so that rotation must be re-expressed
     *    in a frame whose X is the direction the wearer faced at capture:
     *    q' = h⁻¹ ⊗ (q ⊗ q_ref⁻¹) ⊗ h,  h = heading(q_ref).
     *    Without this, a pure physical roll about the "forward" axis reads
     *    as a yaw/pitch/roll mixture whenever the fusion's yaw origin
     *    doesn't happen to align with the capture direction. */
    const Quat q_ref = normalized(current);
    const Quat h = heading(q_ref);

    bore_inv_ = multiply(conjugate(q_ref), h);
    yaw_inv_ = conjugate(h);
}

void Recenterer::recenter(const Quat &current)
{
    /* Zero the yaw of the fully corrected pose (compositional, so repeated
     * recenters and a prior boresight stack). Yaw is a world-frame Z
     * rotation: its inverse composes on the LEFT. */
    const Quat corrected = apply(normalized(current));
    yaw_inv_ = multiply(conjugate(heading(corrected)), yaw_inv_);
}

float Recenterer::set_tilt(const Quat &bore_inv_new, const Quat &current)
{
    const Quat q_c = normalized(current);
    const float phi_old = heading_angle(multiply(q_c, bore_inv_));
    const float phi_new = heading_angle(multiply(q_c, bore_inv_new));
    const float delta = wrap_pi(phi_new - phi_old);

    bore_inv_ = bore_inv_new;
    /* y_new = y_old ⊗ heading(qc⊗b_old) ⊗ conj(heading(qc⊗b_new)):
     * pure yaws commute, so as an angle: ψ ← ψ − δ. Exact in the twist
     * metric (Euler yaw may still move near gimbal poses — by design). */
    set_yaw_angle(wrap_pi(yaw_angle() - delta));
    return delta;
}

float Recenterer::yaw_angle() const
{
    return heading_angle(yaw_inv_);
}

void Recenterer::set_yaw_angle(float rad)
{
    yaw_inv_ = yaw_quat(rad);
}

Quat Recenterer::apply(const Quat &raw) const
{
    return normalized(multiply(yaw_inv_, multiply(raw, bore_inv_)));
}

void Recenterer::reset()
{
    bore_inv_ = {};
    yaw_inv_ = {};
}

} // namespace htk
