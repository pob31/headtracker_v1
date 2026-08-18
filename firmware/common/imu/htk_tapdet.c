/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "htk_tapdet.h"

#include <math.h>

enum {
	ST_IDLE = 0,   /* watching for the first spike */
	ST_SHOCK1,     /* inside the first spike */
	ST_GAP,        /* quiet between taps; t_mark = first spike end */
	ST_REFRACTORY, /* holding off after a detection */
};

void htk_tapdet_init(struct htk_tapdet *d, float odr_hz,
		     const struct htk_tapdet_cfg *cfg)
{
	static const struct htk_tapdet_cfg def = HTK_TAPDET_CFG_DEFAULT;

	d->cfg = cfg ? *cfg : def;
	d->baseline = 9.81f;
	d->dt_nom = 1.0f / odr_hz;
	d->state = ST_IDLE;
	d->t_mark = 0;
	d->t_last = 0;
	d->have_t = false;
	d->primed = false;
	/* two baseline time constants before trusting deviations */
	d->warmup = (uint32_t)(2.0f * d->cfg.baseline_tau_s * odr_hz) + 1u;
}

static uint32_t ms_since(uint32_t now_us, uint32_t then_us)
{
	return (uint32_t)(now_us - then_us) / 1000u; /* wrap-safe */
}

bool htk_tapdet_feed(struct htk_tapdet *d, uint32_t t_us, const float acc[3])
{
	const float mag = sqrtf(acc[0] * acc[0] + acc[1] * acc[1] +
				acc[2] * acc[2]);

	/* EMA baseline. Alpha from the nominal rate: tap timing tolerances are
	 * wide enough that sample-drop jitter is irrelevant here. */
	const float alpha = d->dt_nom / (d->cfg.baseline_tau_s + d->dt_nom);

	d->baseline += alpha * (mag - d->baseline);
	d->t_last = t_us;
	d->have_t = true;

	if (!d->primed) {
		if (d->warmup > 0) {
			d->warmup--;
			return false;
		}
		d->primed = true;
	}

	const float dev = fabsf(mag - d->baseline);
	const bool spike = dev > d->cfg.spike_ths;
	const bool quiet = dev < d->cfg.rearm_ths;
	bool event = false;

	switch (d->state) {
	case ST_IDLE:
		if (spike) {
			d->state = ST_SHOCK1;
			d->t_mark = t_us;
		}
		break;

	case ST_SHOCK1:
		if (quiet) {
			d->state = ST_GAP;
			d->t_mark = t_us; /* gap measured from spike END */
		} else if (ms_since(t_us, d->t_mark) > d->cfg.shock_max_ms) {
			d->state = ST_IDLE; /* sustained shaking: handling */
		}
		break;

	case ST_GAP: {
		const uint32_t gap = ms_since(t_us, d->t_mark);

		if (spike) {
			if (gap < d->cfg.gap_min_ms) {
				/* ringing of the first tap: extend the end */
				d->state = ST_SHOCK1;
				d->t_mark = t_us;
			} else if (gap <= d->cfg.gap_max_ms) {
				event = true;
				d->state = ST_REFRACTORY;
				d->t_mark = t_us;
			} else {
				/* too late: this is a new first tap */
				d->state = ST_SHOCK1;
				d->t_mark = t_us;
			}
		} else if (gap > d->cfg.gap_max_ms) {
			d->state = ST_IDLE; /* single tap, no partner */
		}
		break;
	}

	case ST_REFRACTORY:
		if (ms_since(t_us, d->t_mark) > d->cfg.refractory_ms) {
			d->state = ST_IDLE;
		}
		break;

	default:
		d->state = ST_IDLE;
		break;
	}

	return event;
}
