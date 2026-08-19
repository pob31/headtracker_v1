/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "headtracker/stabilizer.hpp"

#include <cmath>

namespace htk {

namespace {

constexpr float kPi = 3.14159265358979f;
constexpr float kDeg2Rad = kPi / 180.0f;
constexpr float kRad2Deg = 180.0f / kPi;

float vec_norm(const Vec3 &v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec3 vec_scale(const Vec3 &v, float s)
{
    return { v.x * s, v.y * s, v.z * s };
}

/* angle between two (near-)unit vectors, robust for small angles */
float vec_angle(const Vec3 &a, const Vec3 &b)
{
    const float na = vec_norm(a), nb = vec_norm(b);
    if (na < 1e-9f || nb < 1e-9f) {
        return 0.0f;
    }
    const float dot = (a.x * b.x + a.y * b.y + a.z * b.z) / (na * nb);
    const float cx = a.y * b.z - a.z * b.y;
    const float cy = a.z * b.x - a.x * b.z;
    const float cz = a.x * b.y - a.y * b.x;
    const float cross = vec_norm({ cx, cy, cz }) / (na * nb);
    return std::atan2(cross, dot);
}

/* rotation angle of a unit quaternion, in [0, pi] */
float quat_angle(const Quat &q)
{
    float w = std::fabs(q.w);
    if (w > 1.0f) {
        w = 1.0f;
    }
    return 2.0f * std::acos(w);
}

/* geodesic step from a toward b, at most `max_angle` rad (nlerp: the steps
 * are fractions of a degree, where nlerp == slerp to float precision) */
Quat quat_step(const Quat &a, const Quat &b, float max_angle)
{
    Quat to = b;
    float dot = a.w * to.w + a.x * to.x + a.y * to.y + a.z * to.z;
    if (dot < 0.0f) { /* same rotation, opposite sign: take the short way */
        to = { -to.w, -to.x, -to.y, -to.z };
        dot = -dot;
    }
    const Quat diff = multiply(conjugate(a), to);
    const float ang = quat_angle(diff);
    if (ang <= max_angle) {
        return to;
    }
    const float t = max_angle / ang;
    return normalized({ a.w + (to.w - a.w) * t, a.x + (to.x - a.x) * t,
                        a.y + (to.y - a.y) * t, a.z + (to.z - a.z) * t });
}

} // namespace

void Stabilizer::reseed_center(float phi)
{
    z_re_ = std::cos(phi);
    z_im_ = std::sin(phi);
}

void Stabilizer::publish(const StabilizerStatus &s)
{
    /* seqlock: odd = write in progress */
    const uint32_t v = seq_.load(std::memory_order_relaxed);
    seq_.store(v + 1, std::memory_order_release);
    snap_ = s;
    seq_.store(v + 2, std::memory_order_release);
}

StabilizerStatus Stabilizer::status() const
{
    for (;;) {
        const uint32_t a = seq_.load(std::memory_order_acquire);
        if (a & 1u) {
            continue;
        }
        const StabilizerStatus s = snap_;
        const uint32_t b = seq_.load(std::memory_order_acquire);
        if (a == b) {
            return s;
        }
    }
}

Quat Stabilizer::update(const htk_orient &o)
{
    /* ---- sanitize ---- */
    if (!(std::isfinite(o.q_w) && std::isfinite(o.q_x) &&
          std::isfinite(o.q_y) && std::isfinite(o.q_z))) {
        dropped_++;
        return status().q_out; /* last good output, never NaN */
    }
    const Quat q_raw = quat_of(o);

    /* ---- dt ---- */
    float dt = 0.0f;
    if (have_t_) {
        dt = (float) (uint32_t) (o.t_us - last_t_us_) * 1e-6f;
        if (!(dt >= 0.0f) || dt > 0.1f) {
            dt = 0.0f; /* gap or wrap glitch: freeze the time-based math */
        }
    }
    last_t_us_ = o.t_us;
    have_t_ = true;

    /* ---- tap edge ---- */
    const bool tap_now = (o.flags & HTK_ORIENT_TAP) != 0;
    if (!tap_seeded_) {
        /* First sample after start/gap: seed, never fabricate an edge. */
        tap_prev_ = tap_now;
        tap_seeded_ = true;
    }
    if (tap_refractory_left_ > 0.0f) {
        tap_refractory_left_ -= dt;
    }
    if (tap_now && !tap_prev_ && tap_refractory_left_ <= 0.0f) {
        taps_++;
        tap_refractory_left_ = cfg_.tap.refractory_s;
        /* Actions only for a settled wearer: donning headphones is a wear
         * transition full of tap-like jostles. */
        if (cfg_.tap.tap_recenters && worn_ &&
            worn_stable_s_ >= cfg_.tap.min_worn_s) {
            tap_settle_left_ = cfg_.tap.settle_timeout_s;
        }
        if (on_tap) {
            on_tap();
        }
    }
    tap_prev_ = tap_now;

    /* ---- cross-thread requests ---- */
    const uint32_t req = requests_.exchange(0);
    if (req & kReqClear) {
        ref_.reset();
        reseed_center(heading_angle(multiply(q_raw, ref_.tilt())));
    }
    if (req & kReqBoresight) {
        ref_.boresight(q_raw);
        /* manual boresight overrides the estimator: adopt its tilt */
        u_init_ = false;
        reseed_center(heading_angle(multiply(q_raw, ref_.tilt())));
    }
    if (req & kReqRecenter) {
        ref_.recenter(q_raw);
        reseed_center(heading_angle(multiply(q_raw, ref_.tilt())));
    }

    /* ---- wear detection (gravity micro-motion; yaw-blind) ---- */
    const Vec3 g_s = gravity_body(q_raw);
    if (dt > 0.0f) {
        const float a_g = dt / (0.5f + dt); /* 0.5 s reference EMA */
        if (!g_ema_init_) {
            g_ema_ = g_s;
            g_ema_init_ = true;
        } else {
            g_ema_.x += a_g * (g_s.x - g_ema_.x);
            g_ema_.y += a_g * (g_s.y - g_ema_.y);
            g_ema_.z += a_g * (g_s.z - g_ema_.z);
        }
        const float dev = vec_angle(g_s, g_ema_);

        /* a large step = pick-up/put-down: distrust immediately */
        if (dev > 20.0f * kDeg2Rad) {
            worn_ = false;
            worn_time_ = 0.0f;
            still_time_ = 0.0f;
        }

        const float a_w = dt / (cfg_.wear.window_s + dt);
        dev2_ema_ += a_w * (dev * dev - dev2_ema_);
        const bool alive =
            std::sqrt(dev2_ema_) > cfg_.wear.alive_min_deg * kDeg2Rad;

        if (alive) {
            worn_time_ += dt;
            still_time_ = 0.0f;
            if (worn_time_ >= cfg_.wear.worn_hold_s) {
                worn_ = true;
            }
        } else {
            still_time_ += dt;
            worn_time_ = 0.0f;
            if (still_time_ >= cfg_.wear.off_hold_s) {
                worn_ = false;
            }
        }
        worn_stable_s_ = worn_ ? worn_stable_s_ + dt : 0.0f;
    }

    /* ---- auto-level ---- */
    float rate = 0.0f;
    if (have_prev_q_ && dt > 0.0f) {
        rate = quat_angle(multiply(conjugate(prev_q_), q_raw)) / dt;
    }
    prev_q_ = q_raw;
    have_prev_q_ = true;

    /* deferred tap-recenter: wait for the head to settle so "front" is not
     * captured mid-swing; timeout applies it regardless */
    if (tap_settle_left_ >= 0.0f) {
        tap_settle_left_ -= dt;
        if (rate < cfg_.tap.settle_max_deg_s * kDeg2Rad ||
            tap_settle_left_ <= 0.0f) {
            ref_.recenter(q_raw);
            reseed_center(heading_angle(multiply(q_raw, ref_.tilt())));
            tap_settle_left_ = -1.0f;
        }
    }

    if (cfg_.level.enabled && worn_ && dt > 0.0f &&
        rate < cfg_.level.reject_rate_deg_s * kDeg2Rad) {
        const float a_u = dt / (cfg_.level.tau_s + dt);
        if (!u_init_) {
            u_acc_ = g_s;
            u_init_ = true;
        } else {
            u_acc_.x += a_u * (g_s.x - u_acc_.x);
            u_acc_.y += a_u * (g_s.y - u_acc_.y);
            u_acc_.z += a_u * (g_s.z - u_acc_.z);
        }
        worn_total_s_ += dt;

        if (worn_total_s_ >= cfg_.level.warmup_s &&
            vec_norm(u_acc_) > 0.5f) {
            const Quat target = conjugate(shortest_arc_to_down(u_acc_));
            const float tilt = quat_angle(target);

            if (tilt < cfg_.level.max_tilt_deg * kDeg2Rad) {
                const Quat cur = ref_.tilt();
                const float step =
                    cfg_.level.max_slew_deg_s * kDeg2Rad * dt;
                const Quat next = quat_step(cur, target, step);
                const float moved =
                    quat_angle(multiply(conjugate(cur), next));

                if (moved > 1e-7f) {
                    const float delta = ref_.set_tilt(next, q_raw);
                    /* the centering accumulator lives in the pre-yaw
                     * frame, which just moved by delta: co-rotate it */
                    const float c = std::cos(delta), s = std::sin(delta);
                    const float re = z_re_ * c - z_im_ * s;
                    z_im_ = z_re_ * s + z_im_ * c;
                    z_re_ = re;
                    level_updates_++;
                    level_ready_ = true;
                }
            }
        }
    }

    /* ---- auto-center (default off) ---- */
    if (dt > 0.0f) {
        const float phi = heading_angle(multiply(q_raw, ref_.tilt()));
        const float a_m = dt / (cfg_.center.tau_mean_s + dt);
        z_re_ += a_m * (std::cos(phi) - z_re_);
        z_im_ += a_m * (std::sin(phi) - z_im_);
        center_warm_s_ += dt;

        if (cfg_.center.enabled && worn_ &&
            center_warm_s_ >= cfg_.center.tau_mean_s) {
            const float R =
                std::sqrt(z_re_ * z_re_ + z_im_ * z_im_);
            if (R > cfg_.center.min_resultant) {
                const float m = std::atan2(z_im_, z_re_);
                const float err = wrap_pi(-m - ref_.yaw_angle());
                float step = err * dt / cfg_.center.tau_center_s;
                const float cap =
                    cfg_.center.max_slew_deg_s * kDeg2Rad * dt;

                if (step > cap) {
                    step = cap;
                } else if (step < -cap) {
                    step = -cap;
                }
                ref_.set_yaw_angle(wrap_pi(ref_.yaw_angle() + step));
            }
        }
    }

    /* ---- output + snapshot ---- */
    const Quat q_out = ref_.apply(q_raw);

    StabilizerStatus s;
    s.q_out = q_out;
    s.mount_tilt = ref_.tilt();
    s.tilt_deg = quat_angle(ref_.tilt()) * kRad2Deg;
    s.level_confidence = u_init_ ? vec_norm(u_acc_) : 0.0f;
    s.yaw_offset_deg = ref_.yaw_angle() * kRad2Deg;
    s.worn = worn_;
    s.rest = (o.flags & HTK_ORIENT_REST) != 0;
    s.bias_ok = (o.flags & HTK_ORIENT_BIAS_OK) != 0;
    s.level_ready = level_ready_;
    s.taps = taps_;
    s.level_updates = level_updates_;
    s.dropped_nonfinite = dropped_;
    publish(s);

    return q_out;
}

} // namespace htk
