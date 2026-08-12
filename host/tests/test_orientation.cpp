/* Recenter/boresight math per PROTOCOL.md §1.6.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"

#include "headtracker/orientation.hpp"

#include <cmath>
#include <limits>

using htk::Quat;

namespace {

constexpr float kPi = 3.14159265358979f;

Quat from_axis_angle(float ax, float ay, float az, float angle)
{
    const float n = std::sqrt(ax * ax + ay * ay + az * az);
    const float s = std::sin(angle / 2.0f) / n;
    return { std::cos(angle / 2.0f), ax * s, ay * s, az * s };
}

bool approx_same_rotation(const Quat &a, const Quat &b, float tol = 1e-4f)
{
    /* q and -q are the same rotation */
    const float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    return std::fabs(std::fabs(dot) - 1.0f) < tol;
}

} // namespace

TEST_CASE("euler extraction of pure rotations")
{
    const auto yaw90 = from_axis_angle(0, 0, 1, kPi / 2);
    auto e = htk::to_euler(yaw90);
    CHECK(e.yaw == doctest::Approx(kPi / 2).epsilon(1e-3));
    CHECK(e.pitch == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(e.roll == doctest::Approx(0.0f).epsilon(1e-3));

    const auto pitch30 = from_axis_angle(0, 1, 0, kPi / 6);
    e = htk::to_euler(pitch30);
    CHECK(e.pitch == doctest::Approx(kPi / 6).epsilon(1e-3));
}

TEST_CASE("recenter zeroes yaw at the capture pose and preserves pitch/roll")
{
    /* head yawed 70deg and pitched 20deg */
    const auto pose = htk::multiply(from_axis_angle(0, 0, 1, 70 * kPi / 180),
                                    from_axis_angle(0, 1, 0, 20 * kPi / 180));
    htk::Recenterer r;
    r.recenter(pose);

    const auto e = htk::to_euler(r.apply(pose));
    CHECK(e.yaw == doctest::Approx(0.0f).epsilon(1e-3));
    /* yaw-only recenter must keep the gravity-referenced pitch */
    CHECK(e.pitch == doctest::Approx(20 * kPi / 180).epsilon(1e-2));
}

TEST_CASE("recenter is relative: a further 30deg turn reads as 30deg")
{
    const auto pose0 = from_axis_angle(0, 0, 1, 70 * kPi / 180);
    const auto pose1 = htk::multiply(from_axis_angle(0, 0, 1, 30 * kPi / 180), pose0);

    htk::Recenterer r;
    r.recenter(pose0);
    const auto e = htk::to_euler(r.apply(pose1));
    CHECK(e.yaw == doctest::Approx(30 * kPi / 180).epsilon(1e-3));
}

TEST_CASE("boresight absorbs an arbitrary mounting orientation")
{
    /* sensor mounted tilted 25deg roll + 40deg yaw relative to the head */
    const auto mount = htk::multiply(from_axis_angle(1, 0, 0, 25 * kPi / 180),
                                     from_axis_angle(0, 0, 1, 40 * kPi / 180));

    htk::Recenterer r;
    r.boresight(mount); /* captured while wearer is level, facing front */

    /* at the capture pose the corrected output is identity */
    CHECK(approx_same_rotation(r.apply(mount), Quat{}));

    /* a subsequent pure 30deg yaw of the sensor... */
    const auto turned = htk::multiply(from_axis_angle(0, 0, 1, 30 * kPi / 180), mount);
    const auto e = htk::to_euler(r.apply(turned));
    CHECK(e.yaw == doctest::Approx(30 * kPi / 180).epsilon(1e-2));
}

TEST_CASE("recenter composes on top of boresight")
{
    const auto mount = from_axis_angle(1, 0, 0, 15 * kPi / 180);
    htk::Recenterer r;
    r.boresight(mount);

    /* drift: sensor yaws 12deg over time */
    const auto drifted = htk::multiply(from_axis_angle(0, 0, 1, 12 * kPi / 180), mount);
    r.recenter(drifted);
    const auto e = htk::to_euler(r.apply(drifted));
    CHECK(e.yaw == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(e.roll == doctest::Approx(0.0f).epsilon(1e-2)); /* mount still absorbed */
}

TEST_CASE("degenerate wire quaternions normalize to identity, never NaN")
{
    const Quat zero = { 0, 0, 0, 0 };
    const auto n = htk::normalized(zero);
    CHECK(n.w == 1.0f);
    const auto h = htk::heading(from_axis_angle(0, 1, 0, kPi)); /* w=z=0 */
    CHECK(h.w == 1.0f); /* documented degenerate fallback */
}

TEST_CASE("non-finite wire quaternions never propagate")
{
    /* A frame can pass CRC and still carry a NaN/inf float: the CRC covers the
     * bytes, not their meaning. The guards in normalized()/heading() must be
     * written in positive logic, because `n2 < eps` is FALSE for NaN and would
     * let 1/sqrt(NaN) straight through. This is the library's "never emits
     * garbage" contract, and consumers latch filter/reference state from it. */
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const float inf = std::numeric_limits<float>::infinity();

    const Quat bad_quats[] = {
        { nan, 0, 0, 0 }, { 1, nan, 0, 0 }, { 0, 0, nan, 0 }, { 0, 0, 0, nan },
        { inf, 0, 0, 0 }, { 1, 0, inf, 0 }, { nan, nan, nan, nan },
    };

    for (const Quat &bad : bad_quats) {
        const auto n = htk::normalized(bad);
        CHECK(std::isfinite(n.w));
        CHECK(std::isfinite(n.x));
        CHECK(std::isfinite(n.y));
        CHECK(std::isfinite(n.z));

        const auto h = htk::heading(bad);
        CHECK(std::isfinite(h.w));
        CHECK(std::isfinite(h.z));

        /* and through the full reference-frame path a consumer actually uses */
        htk::Recenterer r;
        r.boresight(bad);
        const auto q = r.apply(from_axis_angle(0, 0, 1, kPi / 4));
        CHECK(std::isfinite(q.w));
        CHECK(std::isfinite(q.x));
        CHECK(std::isfinite(q.y));
        CHECK(std::isfinite(q.z));
    }
}

TEST_CASE("quat_of normalizes a non-finite ORIENT payload")
{
    htk_orient o{};
    o.type = HTK_PKT_ORIENT;
    o.q_w = std::numeric_limits<float>::quiet_NaN();
    o.q_x = o.q_y = o.q_z = 0.0f;

    const auto q = htk::quat_of(o);
    CHECK(std::isfinite(q.w));
    CHECK(std::isfinite(q.x));
    CHECK(std::isfinite(q.y));
    CHECK(std::isfinite(q.z));
}
