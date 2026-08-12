/* C-callable wrapper around the vendored VQF filter (single instance).
 * Input units: gyro rad/s, accel m/s^2, body frame X fwd / Y left / Z up.
 * Output: Hamilton quaternion w,x,y,z, body->world (6DoF: yaw origin arbitrary).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_FUSION_H
#define HTK_FUSION_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* sample_hz: rate at which htk_fusion_update() will be called (gyro ODR). */
void htk_fusion_init(float sample_hz);

/* Feed one synchronized gyro+accel sample; fills q[4] = w,x,y,z. */
void htk_fusion_update(const float gyr[3], const float acc[3], float q[4]);

/* True once VQF's online gyro-bias estimate has converged (drives BIAS_OK). */
bool htk_fusion_bias_ok(void);

/* RESET_FUSION command: re-initialize filter state and bias estimate. */
void htk_fusion_reset(void);

#ifdef __cplusplus
}
#endif

#endif
