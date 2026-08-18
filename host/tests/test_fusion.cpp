/* Desktop tests for the FIRMWARE fusion wrapper — htk_fusion.cpp + vendored
 * VQF are Zephyr-free, so the exact code that runs on the head unit runs
 * here against synthetic IMU streams. This is where the yaw-hold servo and
 * rest gate earn trust before touching hardware.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest.h"

extern "C" {
#include "htk_fusion.h"
}

#include <cmath>

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kD2R = kPi / 180.0f;
constexpr float kOdr = 416.0f;
constexpr uint32_t kDtUs = 2404;

float yaw_of(const float q[4])
{
    return 2.0f * std::atan2(q[3], q[0]); /* twist metric, pure-yaw safe */
}

struct Sim {
    uint32_t t = 0;
    float q[4] = { 1, 0, 0, 0 };

    /* still, level, with a constant gyro bias (rad/s) */
    void still(float seconds, const float bias[3])
    {
        const int n = (int) (seconds * kOdr);
        for (int i = 0; i < n; i++) {
            const float gyr[3] = { bias[0], bias[1], bias[2] };
            const float acc[3] = { 0, 0, 9.81f };
            htk_fusion_update(gyr, acc, t, q);
            t += kDtUs;
        }
    }

    /* genuine world-yaw turn at rate_dps, level */
    void turn(float seconds, float rate_dps, const float bias[3])
    {
        const int n = (int) (seconds * kOdr);
        for (int i = 0; i < n; i++) {
            const float gyr[3] = { bias[0], bias[1],
                                   bias[2] + rate_dps * kD2R };
            const float acc[3] = { 0, 0, 9.81f };
            htk_fusion_update(gyr, acc, t, q);
            t += kDtUs;
        }
    }
};

const float kNoBias[3] = { 0, 0, 0 };

} // namespace

TEST_CASE("yaw-hold cancels residual bias drift at rest, without snap-back")
{
    htk_fusion_init(kOdr);
    Sim s;

    /* converge honestly first */
    s.still(15.0f, kNoBias);
    CHECK(htk_fusion_bias_ok());
    CHECK(htk_fusion_rest());
    const float y0 = yaw_of(s.q);

    /* now a residual bias VQF has NOT absorbed appears (0.03 deg/s — the
     * converged-rest floor); the servo must hold yaw against it */
    const float resid[3] = { 0, 0, 0.03f * kD2R };
    float max_dev = 0.0f;
    const int n = (int) (300 * kOdr); /* 5 minutes */
    for (int i = 0; i < n; i++) {
        const float gyr[3] = { resid[0], resid[1], resid[2] };
        const float acc[3] = { 0, 0, 9.81f };
        htk_fusion_update(gyr, acc, s.t, s.q);
        s.t += kDtUs;
        const float dev = std::fabs(yaw_of(s.q) - y0);
        max_dev = std::fmax(max_dev, dev);
    }
    /* without the hold this would be up to 9 deg (0.03 deg/s x 300 s minus
     * what the bias KF absorbs); with it, a fraction of a degree */
    CHECK(max_dev < 0.5f * kD2R);

    /* rest ends: the hold FREEZES — no snap-back jump */
    const float before = yaw_of(s.q);
    s.turn(0.05f, 50.0f, kNoBias); /* motion begins */
    /* the first samples may already integrate real motion; bound the step
     * to what 50 deg/s can physically produce in ~20 samples */
    const float after = yaw_of(s.q);
    CHECK(std::fabs(after - before) < 4.0f * kD2R);
}

TEST_CASE("a genuine turn is tracked >= 90% even if rest was armed")
{
    htk_fusion_init(kOdr);
    Sim s;
    s.still(15.0f, kNoBias);
    CHECK(htk_fusion_rest());

    const float y0 = yaw_of(s.q);
    s.turn(20.0f, 1.0f, kNoBias); /* 1 deg/s for 20 s = 20 deg truth */
    const float moved = std::fabs(yaw_of(s.q) - y0) / kD2R;
    /* gate drops on the first fast sample; worst case the servo stole
     * omega_hold * t = 0.1 * 20 = 2 deg */
    CHECK(moved > 17.5f);
}

TEST_CASE("a constant slow turn is held out of the rest gate while it can be")
{
    htk_fusion_init(kOdr);
    Sim s;
    /* 1.5 deg/s from cold: VQF's own rest detector is fooled (deviation from
     * its LP is ~0 and 1.5 < its 2 deg/s clip). Gate condition 3 (bias-
     * corrected norm > 0.5 deg/s) holds the gate out — but only until VQF's
     * rest-bias KF absorbs the turn into "bias" (its documented loophole,
     * bounded by biasClip = 2 deg/s). Earliest possible arm is restMinT
     * (1.5 s) + our 2 s sustain = 3.5 s, further delayed by absorption time.
     * Assert the hold-out window; what VQF does beyond it is VQF's. */
    for (int i = 0; i < (int) (3.0f * kOdr); i++) {
        const float gyr[3] = { 0, 0, 1.5f * kD2R };
        const float acc[3] = { 0, 0, 9.81f };
        htk_fusion_update(gyr, acc, s.t, s.q);
        s.t += kDtUs;
        CHECK_FALSE(htk_fusion_rest());
    }
}

TEST_CASE("BIAS_OK: meaningful, hysteretic, and honest after reset")
{
    htk_fusion_init(kOdr);
    Sim s;
    CHECK_FALSE(htk_fusion_bias_ok());

    /* Rest-phase convergence is FAST by design (the rest KF's whole point),
     * so no "still not converged after N s" assertion — only the ordering:
     * false at init, true after honest convergence, false again on reset. */
    s.still(20.0f, kNoBias);
    CHECK(htk_fusion_bias_ok());

    htk_fusion_reset();
    CHECK_FALSE(htk_fusion_bias_ok());
    s.still(22.0f, kNoBias);
    CHECK(htk_fusion_bias_ok());
}

TEST_CASE("suspend/resume preserves the bias estimate across standby")
{
    htk_fusion_init(kOdr);
    Sim s;
    const float bias[3] = { 0.2f * kD2R, -0.1f * kD2R, 0.15f * kD2R };

    s.still(25.0f, bias);
    CHECK(htk_fusion_bias_ok());

    htk_fusion_suspend();
    /* minutes pass, unit dark */
    htk_fusion_resume();

    /* restored sigma keeps BIAS_OK from re-converging from scratch */
    s.still(1.0f, bias);
    CHECK(htk_fusion_bias_ok());

    /* and yaw drift with the restored bias stays small over a minute */
    const float y0 = yaw_of(s.q);
    s.still(60.0f, bias);
    CHECK(std::fabs(yaw_of(s.q) - y0) < 1.0f * kD2R);
}

TEST_CASE("quaternion output stays unit-norm over sustained updates")
{
    htk_fusion_init(kOdr);
    Sim s;
    const float bias[3] = { 0, 0, 0.05f * kD2R };
    s.still(30.0f, bias);
    s.turn(10.0f, 20.0f, bias);
    s.still(30.0f, bias);

    const float n = std::sqrt(s.q[0] * s.q[0] + s.q[1] * s.q[1] +
                              s.q[2] * s.q[2] + s.q[3] * s.q[3]);
    CHECK(std::fabs(n - 1.0f) < 1e-5f);
}
