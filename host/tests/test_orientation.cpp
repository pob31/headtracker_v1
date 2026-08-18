/* Recenter/boresight math per PROTOCOL.md §1.6.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"

#include "headtracker/orientation.hpp"

#include <cmath>
#include <initializer_list>
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

namespace {

/* v' = q v q* — rotate a vector by a quaternion (test-local helper) */
void rotate_vec(const Quat &q, const float v[3], float out[3])
{
    const Quat p = { 0.0f, v[0], v[1], v[2] };
    const Quat r = htk::multiply(htk::multiply(q, p), htk::conjugate(q));
    out[0] = r.x;
    out[1] = r.y;
    out[2] = r.z;
}

Quat from_axis_angle_v(const float a[3], float angle)
{
    return from_axis_angle(a[0], a[1], a[2], angle);
}

} // namespace

TEST_CASE("bench sequence: pure physical motions read as pure angles, "
          "independent of the fusion's arbitrary yaw origin")
{
    /* The board lies flat, "cable" (forward) pointing wherever; the fusion's
     * world yaw origin is arbitrary. Regression for a real bench finding:
     * without heading conjugation in the boresight, a pure roll about the
     * forward axis read as a yaw/pitch/roll mixture. */
    for (float origin_deg : { 0.0f, 40.0f, 140.0f, -77.0f }) {
        const Quat q_ref = from_axis_angle(0, 0, 1, origin_deg * kPi / 180);
        htk::Recenterer r;
        r.boresight(q_ref);

        /* at capture: exactly zero */
        CHECK(approx_same_rotation(r.apply(q_ref), Quat{}));

        /* physical forward and ear axes in world coordinates */
        const float fwd_body[3] = { 1, 0, 0 }, ear_body[3] = { 0, 1, 0 };
        float fwd[3], ear[3];
        rotate_vec(q_ref, fwd_body, fwd);
        rotate_vec(q_ref, ear_body, ear);

        /* roll +/-90 about the cable/forward axis: roll only */
        for (float deg : { 90.0f, -90.0f }) {
            const auto q = htk::multiply(from_axis_angle_v(fwd, deg * kPi / 180),
                                         q_ref);
            const auto e = htk::to_euler(r.apply(q));
            CHECK(e.roll == doctest::Approx(deg * kPi / 180).epsilon(1e-3));
            CHECK(e.yaw == doctest::Approx(0.0f).epsilon(1e-3));
            CHECK(e.pitch == doctest::Approx(0.0f).epsilon(1e-3));
        }

        /* nod 30 deg about the ear axis: pitch only */
        const auto nod = htk::multiply(from_axis_angle_v(ear, 30 * kPi / 180),
                                       q_ref);
        const auto en = htk::to_euler(r.apply(nod));
        CHECK(en.pitch == doctest::Approx(30 * kPi / 180).epsilon(1e-3));
        CHECK(en.yaw == doctest::Approx(0.0f).epsilon(1e-3));
        CHECK(en.roll == doctest::Approx(0.0f).epsilon(1e-3));

        /* shake 25 deg about vertical: yaw only */
        const auto shake = htk::multiply(from_axis_angle(0, 0, 1, 25 * kPi / 180),
                                         q_ref);
        const auto es = htk::to_euler(r.apply(shake));
        CHECK(es.yaw == doctest::Approx(25 * kPi / 180).epsilon(1e-3));
        CHECK(es.pitch == doctest::Approx(0.0f).epsilon(1e-3));
        CHECK(es.roll == doctest::Approx(0.0f).epsilon(1e-3));
    }
}

TEST_CASE("KNOWN LIMITATION: mount azimuth cross-couples nod into roll/yaw")
{
    /* Mount azimuth (twist about the vertical) is unobservable from gravity:
     * a single-pose boresight — and auto-level, which sees only gravity —
     * cannot cancel it (3 constraints, 4 unknowns). Pure YAW motion still
     * reads exactly (which is why this stayed invisible); a NOD under an
     * azimuthal mount cross-couples by ~sin(azimuth). This test pins the
     * documented behavior so a change in it is noticed, not discovered.
     * The fix would be motion-based (nod calibration); v1 answers with
     * mechanical keying of the clip. See PROTOCOL.md §1.6. */
    const float az = 30 * kPi / 180;
    const auto mount = from_axis_angle(0, 0, 1, az); /* pure azimuth */
    htk::Recenterer r;
    r.boresight(mount);

    /* a pure 20° nod about the wearer's TRUE ear axis (world Y here) */
    const auto nod = htk::multiply(from_axis_angle(0, 1, 0, 20 * kPi / 180),
                                   mount);
    const auto e = htk::to_euler(r.apply(nod));

    /* yaw motion would read clean; the nod does not: */
    CHECK(std::fabs(e.roll) > 8 * kPi / 180);   /* ~sin(30°)*20° ≈ 10° */
    CHECK(e.pitch < 19 * kPi / 180);            /* pitch under-reads */

    /* while pure yaw under the same mount stays exact: */
    const auto turned = htk::multiply(from_axis_angle(0, 0, 1, 25 * kPi / 180),
                                      mount);
    const auto ey = htk::to_euler(r.apply(turned));
    CHECK(ey.yaw == doctest::Approx(25 * kPi / 180).epsilon(1e-3));
    CHECK(std::fabs(ey.pitch) < 1e-3f);
    CHECK(std::fabs(ey.roll) < 1e-3f);
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
