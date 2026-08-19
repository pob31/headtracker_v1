/* SPDX-License-Identifier: GPL-3.0-or-later
 *
 * VQF wrapper + the drift machinery VQF deliberately does not provide:
 *
 *  - BIAS_OK with hysteresis (VQF's covariance re-inflates when updates stop,
 *    so a single threshold would flap).
 *  - A rest gate stricter than VQF's own: VQF's rest test is deviation from
 *    its own low-pass plus a per-component 2 deg/s clip, which misclassifies
 *    a genuine sustained slow turn as rest. We add a norm test on the
 *    bias-corrected rate and a vertical-component test.
 *  - A slew-limited yaw-hold servo: VQF never freezes integration, so a head
 *    at rest still shows ~0.03 deg/s residual yaw creep. While the gate holds
 *    we walk a persistent LEFT pure-yaw offset toward keeping the output yaw
 *    constant, clamped to HTK_YAWHOLD_SLEW_DPS — a misclassified slow turn
 *    can only ever be eaten at that rate, never wholesale. On gate drop the
 *    offset FREEZES (cancelled drift stays cancelled, no snap-back).
 *
 * The yaw offset is a world-frame (left-side) pure yaw: invisible to
 * body-frame gravity (host auto-level cannot be poisoned by it) and absorbed
 * by the host recenter without any host-side changes.
 */
#include "htk_fusion.h"
#include "vqf/vqf.hpp"

#include <cmath>
#include <new>

/* ---- tunables ---------------------------------------------------------- */

/* Yaw-hold servo slew limit, deg/s. ~3x the 0.03 deg/s residual rest drift;
 * a genuine 1 deg/s turn misclassified as rest loses only 10%. 0 disables the
 * servo while keeping the REST gate observable. */
#ifndef HTK_YAWHOLD_SLEW_DPS
#define HTK_YAWHOLD_SLEW_DPS 0.1f
#endif

#define BIAS_OK_SET_DPS   0.15f
#define BIAS_OK_CLEAR_DPS 0.25f

#define REST_SUSTAIN_S     2.0f
#define REST_GYR_NORM_DPS  0.5f
#define REST_GYR_VERT_DPS  0.3f
#define REST_ACC_DEV_MAX   0.5f

/* Consecutive gate-violating samples that mean genuine motion (~7 ms at
 * 416 Hz). Isolated spike samples shorter than this never drop the gate. */
#define REST_BAD_RUN_DROP 3

/* Post-convergence bias slew clamp, deg/s per second. A CONSTANT-rate slow
 * turn looks like rest to any deviation-from-low-pass detector once the LP
 * settles, and VQF's rest-bias KF then absorbs the whole turn as "bias"
 * (bounded only by its 2 deg/s clip) — a steadily panning listener would
 * silently stop being tracked. Genuine bias moves with temperature at
 * <= ~0.05 deg/s per MINUTE; absorption needs deg/s within seconds. This
 * clamp passes physics and blocks absorption. Applied only after the first
 * honest convergence (initial convergence must stay fast). */
#define BIAS_SLEW_DPS_PER_S 0.01f

static constexpr float kDeg2Rad = 3.14159265358979f / 180.0f;
static constexpr float kPi = 3.14159265358979f;

/* Static storage, placement-new: no heap on target. */
alignas(VQF) static unsigned char vqf_mem[sizeof(VQF)];
static VQF *filt;
static float g_nominal_dt;

static bool g_bias_ok;
static bool g_rest;
static float g_rest_sustain_s;
static uint32_t g_bad_run;

static float g_psi_hold; /* left pure-yaw offset, rad */
static float g_psi_ref;  /* yaw the servo holds toward, rad */
static bool g_reseed;    /* re-seed psi_ref / timers on next sample */

static uint32_t g_last_t_us;
static bool g_have_t;

static bool g_suspended;
static bool g_have_saved_bias;
static vqf_real_t g_saved_bias[3];
static vqf_real_t g_saved_sigma;

static bool g_bias_latched;      /* first honest convergence reached */
static vqf_real_t g_bias_prev[3]; /* last accepted (clamped) bias */

static struct htk_fusion_debug g_dbg; /* window accumulators, see getter */
static uint32_t g_dbg_samples;
static uint32_t g_dbg_vqf_rest;

/* Coarse zero-rate offset, captured over the first second after init/reset.
 * The LSM6DS3TR-C's raw offset can be several deg/s (spec: +/-10), while
 * VQF vetoes rest detection whenever any LOW-PASSED RAW component exceeds
 * its 2 deg/s biasClip — an offset above that permanently blocks rest, the
 * rest-bias KF never runs, and nothing converges (observed on unit 1).
 * Subtracting a coarse boot-time mean keeps VQF's inputs inside every clip;
 * VQF then estimates the residual properly. */
static float g_coarse[3];
static float g_coarse_acc[3];
static float g_coarse_min[3], g_coarse_max[3];
static uint32_t g_coarse_n;
static uint32_t g_coarse_target;
static bool g_coarse_done;

/* Range within the capture window that betrays motion (a still sensor's
 * 1 s peak-to-peak is well under 1 deg/s; handling is tens). */
#define COARSE_RANGE_MAX_DPS  4.0f
/* An accepted offset beyond this is not an offset (spec is +/-10 deg/s). */
#define COARSE_ABS_MAX_DPS    15.0f

static void coarse_restart(void)
{
    g_coarse_acc[0] = g_coarse_acc[1] = g_coarse_acc[2] = 0.0f;
    g_coarse_n = 0;
}

static float wrap_pi(float a)
{
    while (a > kPi) {
        a -= 2.0f * kPi;
    }
    while (a < -kPi) {
        a += 2.0f * kPi;
    }
    return a;
}

/* Twist (heading) angle of q: the world-Z yaw in q = yaw ⊗ tilt.
 * THE metric for all yaw bookkeeping here — not Euler yaw, which disagrees
 * away from level and degenerates at pitch ±90. */
static float heading_angle(const vqf_real_t q[4])
{
    if (std::fabs((float)q[0]) < 1e-6f && std::fabs((float)q[3]) < 1e-6f) {
        return 0.0f; /* gimbal-vertical: heading undefined, hold at 0 */
    }
    return wrap_pi(2.0f * std::atan2((float)q[3], (float)q[0]));
}

/* g_s = conj(q) ⊗ (0,0,-1) ⊗ q : world-down in body coordinates. */
static void gravity_body(const vqf_real_t q[4], float g[3])
{
    const float w = (float)q[0], x = (float)q[1], y = (float)q[2], z = (float)q[3];

    /* third row of R(q), negated (R^T * (0,0,-1)) */
    g[0] = -(2.0f * (x * z - w * y));
    g[1] = -(2.0f * (y * z + w * x));
    g[2] = -(1.0f - 2.0f * (x * x + y * y));
}

void htk_fusion_init(float sample_hz)
{
    g_nominal_dt = 1.0f / sample_hz;
    filt = new (static_cast<void *>(vqf_mem)) VQF(g_nominal_dt);
    /* VQF's rest thresholds stay at their stock values (2 deg/s, 0.5 m/s^2).
     * Hardware showed isolated gyro spikes of 1-5 deg/s even at rest (desk
     * vibration + occasional outlier samples); VQF's rest test is
     * single-sample — any threshold below the spike floor means rest NEVER
     * sustains its 1.5 s dwell and the rest-bias KF never runs. Strictness
     * lives in OUR gate below, which out-votes isolated spikes; protection
     * against VQF absorbing a slow genuine turn into bias lives in the bias
     * slew clamp, which is independent of thresholds. */
    g_bias_ok = false;
    g_rest = false;
    g_rest_sustain_s = 0.0f;
    g_psi_hold = 0.0f;
    g_psi_ref = 0.0f;
    g_reseed = true;
    g_have_t = false;
    g_suspended = false;
    g_have_saved_bias = false;

    g_coarse[0] = g_coarse[1] = g_coarse[2] = 0.0f;
    g_coarse_acc[0] = g_coarse_acc[1] = g_coarse_acc[2] = 0.0f;
    g_coarse_n = 0;
    g_coarse_target = (uint32_t)sample_hz; /* ~1 s */
    g_coarse_done = false;
}

static void update_bias_ok(vqf_real_t sigma_rad)
{
    const float sigma_dps = (float)sigma_rad / kDeg2Rad;

    if (g_bias_ok) {
        if (sigma_dps > BIAS_OK_CLEAR_DPS) {
            g_bias_ok = false;
        }
    } else {
        if (sigma_dps < BIAS_OK_SET_DPS) {
            g_bias_ok = true;
        }
    }
}

void htk_fusion_update(const float gyr[3], const float acc[3], uint32_t t_us,
		       float q[4])
{
    if (!g_coarse_done) {
        /* Accumulate only across a CONSECUTIVE quiet second: the unit is
         * routinely in-hand at boot (always, right after a flash), and a
         * motion-polluted offset is worse than none. Watch the per-axis
         * range within the window; motion restarts the window. */
        if (g_coarse_n == 0) {
            for (int i = 0; i < 3; i++) {
                g_coarse_min[i] = g_coarse_max[i] = gyr[i];
            }
        }
        bool moving = false;

        for (int i = 0; i < 3; i++) {
            g_coarse_min[i] = std::fmin(g_coarse_min[i], gyr[i]);
            g_coarse_max[i] = std::fmax(g_coarse_max[i], gyr[i]);
            if (g_coarse_max[i] - g_coarse_min[i] >
                COARSE_RANGE_MAX_DPS * kDeg2Rad) {
                moving = true;
            }
        }
        if (moving) {
            coarse_restart();
        } else {
            g_coarse_acc[0] += gyr[0];
            g_coarse_acc[1] += gyr[1];
            g_coarse_acc[2] += gyr[2];
            if (++g_coarse_n >= g_coarse_target) {
                const float c0 = g_coarse_acc[0] / (float)g_coarse_n;
                const float c1 = g_coarse_acc[1] / (float)g_coarse_n;
                const float c2 = g_coarse_acc[2] / (float)g_coarse_n;
                const float lim = COARSE_ABS_MAX_DPS * kDeg2Rad;

                if (std::fabs(c0) < lim && std::fabs(c1) < lim &&
                    std::fabs(c2) < lim) {
                    g_coarse[0] = c0;
                    g_coarse[1] = c1;
                    g_coarse[2] = c2;
                    g_coarse_done = true;
                } else {
                    coarse_restart(); /* quiet but absurd: keep waiting */
                }
            }
        }
    }

    vqf_real_t g[3] = { gyr[0] - g_coarse[0], gyr[1] - g_coarse[1],
			gyr[2] - g_coarse[2] };
    vqf_real_t a[3] = { acc[0], acc[1], acc[2] };

    filt->updateGyr(g);
    filt->updateAcc(a);

    vqf_real_t quat[4];
    filt->getQuat6D(quat);

    /* dt for gate/servo timing only (wrap-safe); VQF stays fixed-rate. */
    float dt = g_nominal_dt;
    if (g_have_t) {
        const uint32_t delta = t_us - g_last_t_us;
        dt = (float)delta * 1e-6f;
        if (dt < 0.5f * g_nominal_dt || dt > 4.0f * g_nominal_dt) {
            dt = g_nominal_dt;
        }
    }
    g_last_t_us = t_us;
    g_have_t = true;

    /* ---- bias quality + post-convergence slew clamp ---- */
    vqf_real_t bias[3];
    const vqf_real_t sigma = filt->getBiasEstimate(bias);
    update_bias_ok(sigma);

    if (!g_bias_latched) {
        if (g_bias_ok) {
            g_bias_latched = true;
            g_bias_prev[0] = bias[0];
            g_bias_prev[1] = bias[1];
            g_bias_prev[2] = bias[2];
        }
    } else {
        const float d0 = (float)(bias[0] - g_bias_prev[0]);
        const float d1 = (float)(bias[1] - g_bias_prev[1]);
        const float d2 = (float)(bias[2] - g_bias_prev[2]);
        const float dn = std::sqrt(d0 * d0 + d1 * d1 + d2 * d2);
        const float max_step = BIAS_SLEW_DPS_PER_S * kDeg2Rad * dt;

        if (dn > max_step && dn > 0.0f) {
            const float k = max_step / dn;

            bias[0] = g_bias_prev[0] + (vqf_real_t)(d0 * k);
            bias[1] = g_bias_prev[1] + (vqf_real_t)(d1 * k);
            bias[2] = g_bias_prev[2] + (vqf_real_t)(d2 * k);
            /* sigma < 0 keeps the covariance untouched */
            filt->setBiasEstimate(bias, (vqf_real_t)-1.0);
        }
        g_bias_prev[0] = bias[0];
        g_bias_prev[1] = bias[1];
        g_bias_prev[2] = bias[2];
    }

    /* ---- rest gate ---- */
    /* g[] is already coarse-corrected; bias[] is VQF's residual estimate */
    const float bc[3] = { (float)(g[0] - bias[0]), (float)(g[1] - bias[1]),
			  (float)(g[2] - bias[2]) };
    const float bc_norm =
        std::sqrt(bc[0] * bc[0] + bc[1] * bc[1] + bc[2] * bc[2]);

    float gs[3];
    gravity_body(quat, gs);
    /* gs is unit(ish); vertical rate component = (omega - bias) . (-gs)
     * up-axis; sign irrelevant, magnitude only */
    const float vert =
        std::fabs(bc[0] * gs[0] + bc[1] * gs[1] + bc[2] * gs[2]);

    vqf_real_t rest_dev[2];
    filt->getRelativeRestDeviations(rest_dev);

    const bool vqf_rest = filt->getRestDetected();
    const bool instant_ok = vqf_rest &&
                            bc_norm < REST_GYR_NORM_DPS * kDeg2Rad &&
                            vert < REST_GYR_VERT_DPS * kDeg2Rad;

    /* window diagnostics for htk_fusion_get_debug() */
    g_dbg_samples++;
    if (vqf_rest) {
        g_dbg_vqf_rest++;
    }
    if (!instant_ok) {
        g_dbg.gate_fail_n++;
    }
    if (bc_norm > 1.0f * kDeg2Rad) {
        g_dbg.spikes_1dps++;
        if (bc_norm > 8.0f * kDeg2Rad) {
            g_dbg.spikes_8dps++;
        }
    }
    g_dbg.bc_norm_max_dps = std::fmax(g_dbg.bc_norm_max_dps, bc_norm / kDeg2Rad);
    g_dbg.vert_max_dps = std::fmax(g_dbg.vert_max_dps, vert / kDeg2Rad);
    g_dbg.acc_dev_max = std::fmax(g_dbg.acc_dev_max, (float)rest_dev[1]);
    g_dbg.sigma_dps = (float)sigma / kDeg2Rad;

    if (g_reseed) {
        g_rest_sustain_s = 0.0f;
        g_rest = false;
        g_bad_run = 0;
    }

    /* Motion is SUSTAINED; spikes are ISOLATED (hardware: 1-5 deg/s outlier
     * samples even at rest, from desk vibration and occasional bad reads).
     * Tolerate short violation runs; only a sustained run means motion. */
    if (!instant_ok) {
        g_bad_run++;
        if (g_bad_run >= REST_BAD_RUN_DROP) {
            g_rest_sustain_s = 0.0f;
            g_rest = false;
        }
    } else {
        g_bad_run = 0;
        if (!g_rest) {
            if ((float)rest_dev[1] < REST_ACC_DEV_MAX) {
                g_rest_sustain_s += dt;
                if (g_rest_sustain_s >= REST_SUSTAIN_S) {
                    g_rest = true;
                }
            } else {
                g_rest_sustain_s = 0.0f;
            }
        }
    }

    /* ---- yaw-hold servo (scalar, twist metric) ---- */
    const float phi_fw = heading_angle(quat);
    float psi_out = wrap_pi(g_psi_hold + phi_fw);

    if (g_reseed || !g_rest) {
        g_psi_ref = psi_out; /* track: freeze means "stop integrating" */
    } else {
        const float slew = HTK_YAWHOLD_SLEW_DPS * kDeg2Rad * dt;
        float err = wrap_pi(g_psi_ref - psi_out);

        if (err > slew) {
            err = slew;
        } else if (err < -slew) {
            err = -slew;
        }
        g_psi_hold = wrap_pi(g_psi_hold + err);
        psi_out = wrap_pi(g_psi_hold + phi_fw);
    }
    g_reseed = false;

    /* q_out = yaw(psi_hold) ⊗ q_vqf */
    if (g_psi_hold != 0.0f) {
        const float hw = std::cos(g_psi_hold * 0.5f);
        const float hz = std::sin(g_psi_hold * 0.5f);
        const float w = (float)quat[0], x = (float)quat[1];
        const float y = (float)quat[2], z = (float)quat[3];

        q[0] = hw * w - hz * z;
        q[1] = hw * x - hz * y;
        q[2] = hw * y + hz * x;
        q[3] = hw * z + hz * w;
    } else {
        q[0] = (float)quat[0];
        q[1] = (float)quat[1];
        q[2] = (float)quat[2];
        q[3] = (float)quat[3];
    }

    /* renormalize: this quaternion is consumed by renderers verbatim */
    const float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 > 1e-12f) {
        const float inv = 1.0f / std::sqrt(n2);
        q[0] *= inv;
        q[1] *= inv;
        q[2] *= inv;
        q[3] *= inv;
    } else {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
    }
}

bool htk_fusion_bias_ok(void)
{
    return g_bias_ok;
}

void htk_fusion_get_debug(struct htk_fusion_debug *out)
{
    *out = g_dbg;
    out->coarse_dps[0] = g_coarse[0] / kDeg2Rad;
    out->coarse_dps[1] = g_coarse[1] / kDeg2Rad;
    out->coarse_dps[2] = g_coarse[2] / kDeg2Rad;
    out->vqf_rest_pct = (uint16_t)(g_dbg_samples ?
        (100u * g_dbg_vqf_rest) / g_dbg_samples : 0u);
    /* restart the window */
    g_dbg.bc_norm_max_dps = 0.0f;
    g_dbg.vert_max_dps = 0.0f;
    g_dbg.acc_dev_max = 0.0f;
    g_dbg.gate_fail_n = 0;
    g_dbg.spikes_1dps = 0;
    g_dbg.spikes_8dps = 0;
    g_dbg_samples = 0;
    g_dbg_vqf_rest = 0;
}

void htk_fusion_set_sample_rate(float hz)
{
    const VQFState st = filt->getState();

    g_nominal_dt = 1.0f / hz;
    filt = new (static_cast<void *>(vqf_mem)) VQF(g_nominal_dt);
    filt->setState(st);
    /* thresholds are VQF defaults, nothing further to re-apply */
}

bool htk_fusion_rest(void)
{
    return g_rest;
}

void htk_fusion_reset(void)
{
    filt->resetState();
    g_bias_ok = false;
    g_rest = false;
    g_rest_sustain_s = 0.0f;
    g_psi_hold = 0.0f;
    g_reseed = true;
    g_have_saved_bias = false;
    g_bias_latched = false; /* full reset: convergence starts over */

    /* re-capture the coarse offset too: RESET_FUSION is the recovery for
     * "something is off", and the offset moves with temperature */
    g_coarse_acc[0] = g_coarse_acc[1] = g_coarse_acc[2] = 0.0f;
    g_coarse_n = 0;
    g_coarse_done = false;
}

void htk_fusion_suspend(void)
{
    g_saved_sigma = filt->getBiasEstimate(g_saved_bias);
    g_have_saved_bias = true;
    g_suspended = true;
}

void htk_fusion_resume(void)
{
    if (!g_suspended) {
        return; /* boot-time enter_active(): nothing to restore */
    }
    g_suspended = false;

    /* Attitude and the internal LP/rest filter states are stale after a
     * power-down of unknown length: start clean and re-converge from accel
     * (~tauAcc). The bias, however, is a property of the silicon, not of the
     * pose — restoring it skips the ~10 s rest re-convergence. */
    filt->resetState();
    if (g_have_saved_bias) {
        filt->setBiasEstimate(g_saved_bias, g_saved_sigma);
        g_bias_prev[0] = g_saved_bias[0];
        g_bias_prev[1] = g_saved_bias[1];
        g_bias_prev[2] = g_saved_bias[2];
        /* latch state carries over: the restored bias is the trusted one */
    }
    g_rest = false;
    g_rest_sustain_s = 0.0f;
    g_reseed = true; /* keep psi_hold (still valid), re-seed psi_ref */
}
