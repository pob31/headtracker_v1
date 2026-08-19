/* Software double-tap detector on the accelerometer stream.
 *
 * Chosen over the LSM6DS3TR-C's embedded tap engine deliberately: the Zephyr
 * lsm6dsl driver exposes no tap support (and owns INT1), the register path
 * couples to ODR/standby and carries unverifiable silicon questions, and a
 * software detector is IMU-agnostic (survives the LSM6DSV16X swap) and
 * unit-testable on the host with synthetic waveforms.
 *
 * Signature: a short jerk spike (tap shock), a quiet gap, a second spike —
 * then a refractory period. All thresholds are struct fields so the bench can
 * tune without recompiling callers.
 *
 * Pure C99, no OS dependencies. One instance per IMU stream.
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef HTK_TAPDET_H
#define HTK_TAPDET_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct htk_tapdet_cfg {
	/* deviation of |acc| from its short EMA that counts as a spike, m/s^2.
	 * A finger tap on a rigid shell is typically > 10; head motion < 3. */
	float spike_ths;
	/* spike is over once deviation falls below this, m/s^2 */
	float rearm_ths;
	/* EMA time constant for the |acc| baseline, seconds (~40 ms) */
	float baseline_tau_s;
	/* a "shock" longer than this is handling, not a tap, ms */
	uint16_t shock_max_ms;
	/* second tap must start within [gap_min, gap_max] of the first
	 * spike's END, ms. Below gap_min it is ringing of the first tap. */
	uint16_t gap_min_ms;
	uint16_t gap_max_ms;
	/* ignore everything for this long after a detection, ms */
	uint16_t refractory_ms;
};

/* Bench-tuned defaults (416 Hz). spike_ths raised 8 -> 18 m/s^2 after
 * hardware showed a brisk hand jerk (accelerate + decelerate ~100-300 ms
 * apart) reads as a fake double tap at 8; sharp finger taps on the shell
 * comfortably exceed 18. */
#define HTK_TAPDET_CFG_DEFAULT                                                 \
	{                                                                      \
		.spike_ths = 18.0f, .rearm_ths = 5.0f,                         \
		.baseline_tau_s = 0.04f,                                       \
		.shock_max_ms = 60, .gap_min_ms = 70, .gap_max_ms = 500,       \
		.refractory_ms = 1000,                                         \
	}

struct htk_tapdet {
	struct htk_tapdet_cfg cfg;
	float baseline;    /* EMA of |acc| */
	float dt_nom;      /* 1/odr, for the EMA coefficient */
	uint8_t state;     /* internal state machine */
	uint32_t t_mark;   /* state-entry / spike-end timestamp, us */
	uint32_t t_last;   /* previous sample timestamp, us */
	bool have_t;
	bool primed;       /* baseline warmed up */
	uint32_t warmup;   /* samples until primed */
};

void htk_tapdet_init(struct htk_tapdet *d, float odr_hz,
		     const struct htk_tapdet_cfg *cfg /* NULL = defaults */);

/* Feed one accel sample (m/s^2, any frame — only |acc| is used).
 * Returns true exactly once per detected double tap. */
bool htk_tapdet_feed(struct htk_tapdet *d, uint32_t t_us, const float acc[3]);

#ifdef __cplusplus
}
#endif

#endif
