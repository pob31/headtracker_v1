/* htk::Stabilizer — per-tracker reference-frame automation.
 *
 * Owns a Recenterer and layers on top of it:
 *   - AUTO-LEVEL: the pitch/roll mounting tilt self-calibrates from the
 *     long-term average of body-frame gravity while the unit is worn. No
 *     capture ritual, and headphones resting in odd positions cannot corrupt
 *     it (the wear gate excludes them). Every tilt refinement re-anchors yaw
 *     so the output never jumps. Mount AZIMUTH (twist about vertical) is
 *     fundamentally unobservable from gravity — see PROTOCOL.md §1.6.
 *   - WEAR DETECTION: worn heads show micro-motion (breathing, sway); desks
 *     are dead still. Measured on gravity-tilt deviations, which are blind to
 *     every yaw manipulation by construction — the firmware's rest yaw-hold
 *     cannot blind this detector.
 *   - AUTO-CENTER (default OFF): leaks yaw toward the long-term mean heading
 *     so drift self-corrects for a listener who faces front on average.
 *   - TAP: edge-detects HTK_ORIENT_TAP and (by default) recenters.
 *
 * Threading contract (mirrors htk::Client): update() runs on the reader
 * thread and never blocks or allocates. request_*() and status() are safe
 * from any thread (atomic bitmask in, seqlock snapshot out). on_tap fires on
 * the reader thread and must not block.
 *
 * ONE INSTANCE PER TRACKER. Feeding two ids through one instance silently
 * mixes reference frames; filter by id first (see htmon for the pattern).
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>

#include "orientation.hpp"

namespace htk {

struct AutoLevelConfig {
    bool enabled = true;
    float tau_s = 120.0f;          /* gravity-average time constant */
    float warmup_s = 20.0f;        /* worn time before the first correction */
    float max_slew_deg_s = 0.2f;   /* tilt correction rate limit */
    float max_tilt_deg = 45.0f;    /* estimates beyond this are distrusted */
    float reject_rate_deg_s = 60.0f; /* skip samples during fast motion */
};

struct AutoCenterConfig {
    bool enabled = false;          /* deliberate: a WFS scene should not
                                    * rotate unless the listener asked */
    float tau_mean_s = 25.0f;      /* circular-mean estimator */
    float tau_center_s = 90.0f;    /* correction leak */
    float max_slew_deg_s = 0.2f;   /* must exceed the drift rate (~0.1°/s) */
    float min_resultant = 0.5f;    /* mean confidence gate, 0..1 */
};

struct WearConfig {
    float window_s = 4.0f;         /* micro-motion RMS window */
    float alive_min_deg = 0.02f;   /* TUNE ON HARDWARE: desk ~1e-3°, worn
                                    * 0.05-0.5° (see plan: measured captures
                                    * before trusting this number) */
    float worn_hold_s = 3.0f;      /* sustained aliveness before "worn" */
    float off_hold_s = 10.0f;      /* sustained stillness before "off" */
};

struct TapConfig {
    bool tap_recenters = true;
    float refractory_s = 1.0f;     /* host-side, on top of the firmware's */
};

struct StabilizerConfig {
    AutoLevelConfig level;
    AutoCenterConfig center;
    WearConfig wear;
    TapConfig tap;
};

struct StabilizerStatus {
    Quat q_out;                    /* last corrected orientation */
    Quat mount_tilt;               /* current tilt correction (bore_inv) */
    float tilt_deg = 0.0f;         /* magnitude of the mount tilt estimate */
    float level_confidence = 0.0f; /* |gravity EMA|: 1 = consistent */
    float yaw_offset_deg = 0.0f;   /* current recenter/auto-center yaw */
    bool worn = false;
    bool rest = false;             /* HTK_ORIENT_REST from the last sample */
    bool bias_ok = false;
    bool level_ready = false;      /* auto-level has begun correcting */
    uint32_t taps = 0;
    uint32_t level_updates = 0;
    uint32_t dropped_nonfinite = 0;
};

class Stabilizer {
public:
    Stabilizer() = default;
    explicit Stabilizer(const StabilizerConfig &cfg) : cfg_(cfg) {}

    void configure(const StabilizerConfig &cfg) { cfg_ = cfg; }
    const StabilizerConfig &config() const { return cfg_; }

    /* Reader thread only. Returns the reference-corrected orientation. */
    Quat update(const htk_orient &o);

    /* Any thread. Applied at the start of the next update(). */
    void request_recenter() { requests_.fetch_or(kReqRecenter); }
    void request_boresight() { requests_.fetch_or(kReqBoresight); }
    void request_clear() { requests_.fetch_or(kReqClear); }

    /* Any thread. Consistent snapshot via seqlock. */
    StabilizerStatus status() const;

    /* Fires on the reader thread on a detected tap edge; must not block. */
    std::function<void()> on_tap;

private:
    static constexpr uint32_t kReqRecenter = 1u << 0;
    static constexpr uint32_t kReqBoresight = 1u << 1;
    static constexpr uint32_t kReqClear = 1u << 2;

    void reseed_center(float phi);
    void publish(const StabilizerStatus &s);

    StabilizerConfig cfg_;
    Recenterer ref_;

    /* --- reader-thread state --- */
    bool have_t_ = false;
    uint32_t last_t_us_ = 0;
    bool have_prev_q_ = false;
    Quat prev_q_;
    bool tap_prev_ = false;
    bool tap_seeded_ = false;
    float tap_refractory_left_ = 0.0f;

    /* wear */
    Vec3 g_ema_ {};
    bool g_ema_init_ = false;
    float dev2_ema_ = 0.0f;        /* mean squared tilt deviation, rad^2 */
    float worn_time_ = 0.0f;
    float still_time_ = 0.0f;
    bool worn_ = false;

    /* auto-level */
    Vec3 u_acc_ {};                /* gravity direction EMA (unnormalized) */
    bool u_init_ = false;
    float worn_total_s_ = 0.0f;
    bool level_ready_ = false;
    uint32_t level_updates_ = 0;

    /* auto-center */
    float z_re_ = 0.0f, z_im_ = 0.0f; /* circular mean accumulator */
    float center_warm_s_ = 0.0f;

    uint32_t taps_ = 0;
    uint32_t dropped_ = 0;

    /* cross-thread */
    std::atomic<uint32_t> requests_ { 0 };
    mutable std::atomic<uint32_t> seq_ { 0 };
    StabilizerStatus snap_ {};
};

} // namespace htk
