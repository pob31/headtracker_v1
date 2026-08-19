/* Stabilizer math: auto-level, yaw re-anchoring, auto-center, wear, tap.
 * The invariants here encode the design review's verified algebra — several
 * assert AGAINST plausible-but-wrong formulations we almost shipped.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"

#include "headtracker/stabilizer.hpp"

#include <cmath>
#include <initializer_list>

using htk::Quat;
using htk::Vec3;

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kD2R = kPi / 180.0f;

Quat aa(float x, float y, float z, float angle)
{
    const float n = std::sqrt(x * x + y * y + z * z);
    const float s = std::sin(angle / 2.0f) / n;
    return { std::cos(angle / 2.0f), x * s, y * s, z * s };
}

htk_orient orient_of(const Quat &q, uint32_t t_us, uint8_t flags = 0)
{
    htk_orient o {};
    o.type = HTK_PKT_ORIENT;
    o.id = 0x1234;
    o.t_us = t_us;
    o.q_w = q.w;
    o.q_x = q.x;
    o.q_y = q.y;
    o.q_z = q.z;
    o.flags = flags;
    return o;
}

/* xorshift, deterministic across platforms */
struct Rng {
    uint64_t s = 0x1234567;
    float uniform() /* [-1, 1) */
    {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return (float) (int32_t) (s * 0x2545F4914F6CDD1Dull >> 33) / 1073741824.0f;
    }
};

/* Fast-converging test config: real-world taus are minutes; tests compress. */
htk::StabilizerConfig fast_cfg()
{
    htk::StabilizerConfig c;
    c.level.tau_s = 3.0f;
    c.level.warmup_s = 0.5f;
    c.level.max_slew_deg_s = 60.0f;
    c.wear.alive_min_deg = -1.0f; /* stub: always "worn" after the hold */
    c.wear.worn_hold_s = 0.1f;
    return c;
}

constexpr uint32_t kDt208 = 4808; /* us, ~208 Hz */

} // namespace

TEST_CASE("gravity_body is invariant under every world-yaw manipulation")
{
    Rng rng;
    const Quat pose = htk::multiply(aa(0, 1, 0, 20 * kD2R), aa(1, 0, 0, -8 * kD2R));
    const Vec3 base = htk::gravity_body(pose);

    for (int i = 0; i < 100; i++) {
        const float yaw = rng.uniform() * kPi;
        const Vec3 g = htk::gravity_body(htk::multiply(htk::yaw_quat(yaw), pose));
        CHECK(std::fabs(g.x - base.x) < 1e-5f);
        CHECK(std::fabs(g.y - base.y) < 1e-5f);
        CHECK(std::fabs(g.z - base.z) < 1e-5f);
    }
}

TEST_CASE("shortest_arc_to_down: exact, pure-swing, degenerate-safe, sign-symmetric")
{
    Rng rng;
    for (int i = 0; i < 200; i++) {
        Vec3 g = { rng.uniform(), rng.uniform(), rng.uniform() };
        if (std::fabs(g.x) + std::fabs(g.y) + std::fabs(g.z) < 0.1f) {
            continue;
        }
        const Quat q = htk::shortest_arc_to_down(g);

        /* pure swing: z exactly zero by construction */
        CHECK(q.z == 0.0f);

        /* maps g to (0,0,-1) */
        const float n = std::sqrt(g.x * g.x + g.y * g.y + g.z * g.z);
        const Quat p = { 0.0f, g.x / n, g.y / n, g.z / n };
        const Quat r = htk::multiply(htk::multiply(q, p), htk::conjugate(q));
        CHECK(std::fabs(r.x) < 1e-5f);
        CHECK(std::fabs(r.y) < 1e-5f);
        CHECK(r.z == doctest::Approx(-1.0f).epsilon(1e-4));
    }

    /* degenerate: g straight up (upside-down mount) */
    const Quat up = htk::shortest_arc_to_down({ 0, 0, 1 });
    CHECK(std::isfinite(up.w));
    const Quat p = { 0, 0, 0, 1 };
    const Quat r = htk::multiply(htk::multiply(up, p), htk::conjugate(up));
    CHECK(r.z == doctest::Approx(-1.0f).epsilon(1e-4));

    /* joint sign symmetry: -g with up-convention == g with down-convention */
    const Vec3 g = { 0.3f, -0.2f, -0.9f };
    const Quat a = htk::shortest_arc_to_down(g);
    const Quat b = htk::shortest_arc_to_down({ -g.x, -g.y, -g.z });
    /* b maps -g to down, i.e. g to up: composing must give a 180° flip, not
     * equality — mixing the conventions is the trap; assert they differ. */
    const float dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
    CHECK(std::fabs(dot) < 0.999f);
}

TEST_CASE("set_tilt re-anchors yaw exactly in the twist metric")
{
    /* NOTE deliberately NOT asserted: to_euler(...).yaw continuity. The
     * re-anchor is exact in heading_angle() and only there; Euler yaw moves
     * near tilted poses by design. Do not "fix" the formula to make an
     * Euler-based assertion pass — that is the recurring bug class. */
    Rng rng;
    for (int i = 0; i < 100; i++) {
        htk::Recenterer ref;
        const Quat pose =
            htk::multiply(htk::yaw_quat(rng.uniform() * kPi),
                          htk::multiply(aa(0, 1, 0, rng.uniform() * 0.6f),
                                        aa(1, 0, 0, rng.uniform() * 0.6f)));
        ref.recenter(pose);
        const float before = htk::heading_angle(ref.apply(pose));

        const Quat new_tilt =
            htk::conjugate(htk::shortest_arc_to_down(htk::gravity_body(
                htk::multiply(aa(1, 0, 0, rng.uniform() * 0.3f), pose))));
        ref.set_tilt(new_tilt, pose);
        const float after = htk::heading_angle(ref.apply(pose));

        CHECK(htk::wrap_pi(after - before) == doctest::Approx(0.0f).epsilon(2e-4));
    }
}

TEST_CASE("a 20-degree tilt step without re-anchoring would jump the yaw")
{
    /* the regression proving set_tilt earns its keep */
    const Quat pose = htk::multiply(htk::yaw_quat(1.0f), aa(0, 1, 0, 0.7f));
    htk::Recenterer with, without;
    with.recenter(pose);
    without.recenter(pose);

    const Quat step = htk::conjugate(aa(1, 0, 0, 20 * kD2R));
    const float base = htk::heading_angle(with.apply(pose));

    with.set_tilt(step, pose);
    const float anchored = htk::heading_angle(with.apply(pose));
    CHECK(std::fabs(htk::wrap_pi(anchored - base)) < 1e-3f);

    /* naive: swap the tilt in with no yaw compensation */
    const float naive = htk::heading_angle(
        htk::multiply(htk::yaw_quat(without.yaw_angle()),
                      htk::multiply(pose, step)));
    CHECK(std::fabs(htk::wrap_pi(naive - base)) > 2 * kD2R);
}

TEST_CASE("auto-level converges to the mount tilt, yaw-drift-proof")
{
    /* mount: 18° roll + 25° azimuth (azimuth must NOT corrupt the tilt);
     * head: upright on average with noisy nodding; world yaw: drifting. */
    const Quat mount = htk::multiply(aa(1, 0, 0, 18 * kD2R),
                                     htk::yaw_quat(25 * kD2R));
    htk::StabilizerConfig cfg = fast_cfg();
    /* tau vs assertion: the 0.4 Hz nod leaves ~6°/(2π·0.4·tau) of ripple on
     * the gravity EMA — tau 6 s puts it at 0.4°, inside the 0.6° bound. */
    cfg.level.tau_s = 6.0f;
    htk::Stabilizer stab(cfg);
    Rng rng;
    uint32_t t = 0;

    for (int i = 0; i < 208 * 45; i++) {
        const float ts = t * 1e-6f;
        const float drift = 0.5f * kD2R * ts; /* yaw ramp */
        /* smooth, physiological motion: random per-sample noise would look
         * like >1000 deg/s to the motion-reject gate */
        const Quat head = htk::multiply(
            aa(0, 1, 0, 6 * kD2R * std::sin(2 * kPi * 0.4f * ts)),
            aa(1, 0, 0, 4 * kD2R * std::sin(2 * kPi * 0.7f * ts + 1.0f)));
        const Quat q =
            htk::multiply(htk::yaw_quat(drift),
                          htk::multiply(head, mount));
        stab.update(orient_of(q, t));
        t += kDt208;
    }

    const auto st = stab.status();
    CHECK(st.level_ready);

    /* the estimated tilt must cancel the mount's SWING (18°), leaving only
     * the unobservable azimuth: corrected upright pose has level gravity */
    const Quat upright = htk::multiply(htk::yaw_quat(0.3f), mount);
    htk_orient probe = orient_of(upright, t);
    const Quat corrected = stab.update(probe);
    const Vec3 g = htk::gravity_body(corrected);
    const float residual_deg =
        std::acos(std::fmax(-1.0f, std::fmin(1.0f, -g.z))) / kD2R;
    CHECK(residual_deg < 0.6f);
}

TEST_CASE("auto-level learns the habitual posture as level (documented bias)")
{
    /* wearer sits pitched 15° down on average: the estimator can't know
     * better — assert the bias so it stays a known number */
    htk::Stabilizer stab(fast_cfg());
    Rng rng;
    uint32_t t = 0;
    const Quat posture = aa(0, 1, 0, 15 * kD2R);

    for (int i = 0; i < 208 * 20; i++) {
        const float ts = t * 1e-6f;
        const Quat head = htk::multiply(
            posture, aa(0, 1, 0, 3 * kD2R * std::sin(2 * kPi * 0.5f * ts)));
        stab.update(orient_of(head, t));
        t += kDt208;
    }

    /* at the true-level pose the output now reads pitched UP by ~15° */
    const auto e = htk::to_euler(stab.update(orient_of({}, t)));
    CHECK(e.pitch == doctest::Approx(-15 * kD2R).epsilon(0.15));
}

TEST_CASE("wear detector: desk still, worn alive, and yaw-blind")
{
    htk::StabilizerConfig cfg; /* real thresholds this time */
    cfg.level.enabled = false;
    htk::Stabilizer desk(cfg), worn(cfg), worn_yawing(cfg);
    Rng rng;
    uint32_t t = 0;

    for (int i = 0; i < 208 * 15; i++) {
        const float ts = t * 1e-6f;

        desk.update(orient_of(aa(0, 1, 0, 0.001f * kD2R * rng.uniform()), t));

        /* breathing 0.05° @ 1.2 Hz + sway 0.2° @ 0.3 Hz */
        const Quat sway = htk::multiply(
            aa(0, 1, 0, 0.2f * kD2R * std::sin(2 * kPi * 0.3f * ts)),
            aa(1, 0, 0, 0.05f * kD2R * std::sin(2 * kPi * 1.2f * ts)));
        worn.update(orient_of(sway, t));

        /* same micro-motion under a fast yaw ramp: MUST make no difference
         * (the firmware yaw-hold zeroes yaw deltas; a detector relying on
         * them would be blinded) */
        worn_yawing.update(orient_of(
            htk::multiply(htk::yaw_quat(30 * kD2R * ts), sway), t));

        t += kDt208;
    }

    CHECK_FALSE(desk.status().worn);
    CHECK(worn.status().worn);
    CHECK(worn_yawing.status().worn);
}

TEST_CASE("auto-center: converges, bounded slew, resets exactly on recenter")
{
    htk::StabilizerConfig cfg = fast_cfg();
    cfg.level.enabled = false;
    cfg.center.enabled = true;
    cfg.center.tau_mean_s = 2.0f;
    cfg.center.tau_center_s = 6.0f;
    cfg.center.max_slew_deg_s = 5.0f;
    htk::Stabilizer stab(cfg);
    uint32_t t = 0;

    /* constant drift r: after settling, |output yaw| ≈ r * tau_center */
    const float r = 0.5f * kD2R; /* rad/s */
    float prev_yaw = 0.0f;
    float max_step = 0.0f;
    for (int i = 0; i < 208 * 60; i++) {
        const Quat q = htk::yaw_quat(r * (t * 1e-6f));
        const Quat out = stab.update(orient_of(q, t));
        const float y = htk::heading_angle(out);
        if (i > 208) {
            max_step = std::fmax(max_step,
                                 std::fabs(htk::wrap_pi(y - prev_yaw)));
        }
        prev_yaw = y;
        t += kDt208;
    }
    const float ess = r * cfg.center.tau_center_s;
    CHECK(std::fabs(prev_yaw) < ess * 1.5f);
    CHECK(std::fabs(prev_yaw) > ess * 0.4f);
    /* output never moves faster than drift + slew cap per sample */
    const float cap = (r + cfg.center.max_slew_deg_s * kD2R) * 0.006f;
    CHECK(max_step < cap * 1.5f);

    /* recenter: error must be exactly zero and stay near zero briefly */
    stab.request_recenter();
    const Quat q0 = htk::yaw_quat(r * (t * 1e-6f));
    const Quat out0 = stab.update(orient_of(q0, t));
    CHECK(std::fabs(htk::heading_angle(out0)) < 1e-3f);
    t += kDt208;
    for (int i = 0; i < 208; i++) { /* 1 s */
        stab.update(orient_of(htk::yaw_quat(r * (t * 1e-6f)), t));
        t += kDt208;
    }
    /* within 1 s the leak may move at most slew*1s + drift accumulation */
    const float y1 = htk::heading_angle(stab.status().q_out);
    CHECK(std::fabs(y1) < (r + cfg.center.max_slew_deg_s * kD2R) * 1.2f);
}

TEST_CASE("auto-center survives ±pi wraparound without 360-degree unwinds")
{
    htk::StabilizerConfig cfg = fast_cfg();
    cfg.level.enabled = false;
    cfg.center.enabled = true;
    cfg.center.tau_mean_s = 1.0f;
    cfg.center.tau_center_s = 3.0f;
    htk::Stabilizer stab(cfg);
    uint32_t t = 0;
    float prev = 0.0f;

    for (int i = 0; i < 208 * 20; i++) {
        /* oscillate across the ±pi seam */
        const float yaw = kPi + 0.3f * std::sin(2 * kPi * 0.5f * t * 1e-6f);
        const Quat out = stab.update(orient_of(htk::yaw_quat(yaw), t));
        const float y = htk::heading_angle(out);
        if (i > 0) {
            CHECK(std::fabs(htk::wrap_pi(y - prev)) < 0.5f);
        }
        prev = y;
        t += kDt208;
    }
}

TEST_CASE("tap edge detection: one event per hold, gap-start never fabricates")
{
    htk::StabilizerConfig cfg = fast_cfg();
    cfg.level.enabled = false;
    int events = 0;
    htk::Stabilizer stab(cfg);
    stab.on_tap = [&] { events++; };
    uint32_t t = 0;

    auto feed = [&](int frames, uint8_t flags) {
        for (int i = 0; i < frames; i++) {
            stab.update(orient_of({}, t, flags));
            t += kDt208;
        }
    };

    /* stream STARTS with TAP held (reconnect mid-hold): no event */
    feed(52, HTK_ORIENT_TAP);
    CHECK(events == 0);
    feed(208, 0);

    /* a normal 250 ms hold: exactly one event */
    feed(52, HTK_ORIENT_TAP);
    feed(104, 0);
    CHECK(events == 1);

    /* two holds 50 ms apart: host refractory folds them into one */
    feed(52, HTK_ORIENT_TAP);
    feed(10, 0);
    feed(52, HTK_ORIENT_TAP);
    CHECK(events == 2); /* first of the pair counts... */
    feed(300, 0);

    /* unknown future bits (5-7) change nothing */
    feed(52, (uint8_t) (1u << 6));
    CHECK(events == 2);
}

TEST_CASE("tap default action recenters the yaw (worn and settled)")
{
    htk::StabilizerConfig cfg = fast_cfg();
    cfg.level.enabled = false;
    cfg.tap.min_worn_s = 0.2f;
    htk::Stabilizer stab(cfg);
    uint32_t t = 0;

    const Quat pose = htk::yaw_quat(70 * kD2R);
    for (int i = 0; i < 208; i++) { /* 1 s: worn + stable */
        stab.update(orient_of(pose, t, 0));
        t += kDt208;
    }
    CHECK(std::fabs(htk::heading_angle(stab.status().q_out) - 70 * kD2R) < 1e-3f);

    stab.update(orient_of(pose, t, HTK_ORIENT_TAP)); /* edge */
    t += kDt208;
    const Quat out = stab.update(orient_of(pose, t, HTK_ORIENT_TAP));
    CHECK(std::fabs(htk::heading_angle(out)) < 1e-3f);
}

TEST_CASE("tap during a wear transition (donning) does NOT recenter")
{
    htk::StabilizerConfig cfg = fast_cfg();
    cfg.level.enabled = false;
    cfg.tap.min_worn_s = 2.0f; /* real default: settled wearers only */
    int events = 0;
    htk::Stabilizer stab(cfg);
    stab.on_tap = [&] { events++; };
    uint32_t t = 0;

    const Quat pose = htk::yaw_quat(50 * kD2R);
    /* only ~0.5 s of wear before the tap: still in the donning window */
    for (int i = 0; i < 104; i++) {
        stab.update(orient_of(pose, t, 0));
        t += kDt208;
    }
    stab.update(orient_of(pose, t, HTK_ORIENT_TAP));
    t += kDt208;
    for (int i = 0; i < 52; i++) {
        stab.update(orient_of(pose, t, HTK_ORIENT_TAP));
        t += kDt208;
    }

    CHECK(events == 1); /* observability preserved... */
    CHECK(std::fabs(htk::heading_angle(stab.status().q_out) - 50 * kD2R) <
          1e-3f); /* ...but no action taken */
}
