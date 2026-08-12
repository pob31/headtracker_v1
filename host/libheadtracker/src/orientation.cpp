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
    /* RIGHT-side inverse: for body->world quaternions the mounting offset
     * composes on the body side, q_sensor = q_head ⊗ q_mount. A left-side
     * tare would conjugate later rotations into the tilted mount axes
     * (a 30° yaw would read ~28° under a 25° mount roll); right-side tare
     * cancels the mount exactly: q ⊗ q_ref⁻¹ = q_worldmotion. */
    bore_inv_ = conjugate(normalized(current));
    yaw_inv_ = {};
}

void Recenterer::recenter(const Quat &current)
{
    /* Zero the yaw of the boresight-corrected pose, not of the raw sensor
     * pose — so recenter composes with a prior boresight. Yaw is a
     * world-frame Z rotation, so its inverse applies on the LEFT. */
    const Quat corrected = multiply(normalized(current), bore_inv_);
    yaw_inv_ = conjugate(heading(corrected));
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
