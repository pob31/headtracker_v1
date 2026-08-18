/* C-callable wrapper around the vendored VQF filter (single instance).
 * Input units: gyro rad/s, accel m/s^2, body frame X fwd / Y left / Z up.
 * Output: Hamilton quaternion w,x,y,z, body->world (6DoF: yaw origin arbitrary,
 * and may be re-anchored by the internal rest yaw-hold — ORIENT yaw is NOT the
 * integral of the raw gyro, see PROTOCOL.md §1.3).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_FUSION_H
#define HTK_FUSION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* sample_hz: rate at which htk_fusion_update() will be called (gyro ODR). */
void htk_fusion_init(float sample_hz);

/* Feed one synchronized gyro+accel sample; fills q[4] = w,x,y,z.
 * t_us: capture timestamp (wraps at 2^32) — used for gate/servo timing only;
 * VQF itself integrates at the fixed configured rate by design. */
void htk_fusion_update(const float gyr[3], const float acc[3], uint32_t t_us,
		       float q[4]);

/* True once the gyro-bias estimate has genuinely converged (sigma < 0.15
 * deg/s, released at 0.25 deg/s — hysteresis because VQF re-inflates the
 * covariance when updates stop). Drives HTK_ORIENT_BIAS_OK. */
bool htk_fusion_bias_ok(void);

/* Drift-gate result (drives HTK_ORIENT_REST): the unit considers itself at
 * rest — VQF rest detection sustained >= 2 s, bias-corrected gyro norm below
 * 0.5 deg/s (rejects slow genuine turns VQF's own gate misses), its vertical
 * component below 0.3 deg/s, and the accel rest deviation comfortable. While
 * true, the yaw-hold servo (if enabled) is cancelling residual yaw drift. */
bool htk_fusion_rest(void);

/* RESET_FUSION command: full re-initialization, bias estimate included
 * (spec-mandated observable behavior — BIAS_OK drops and re-converges). */
void htk_fusion_reset(void);

/* Standby transitions: suspend saves the converged bias estimate; resume
 * re-initializes the filter state (attitude re-converges from accel within
 * ~tauAcc) but restores the bias, so drift performance survives a power-down
 * without the ~10 s rest re-convergence. */
void htk_fusion_suspend(void);
void htk_fusion_resume(void);

#ifdef __cplusplus
}
#endif

#endif
