/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "htk_fusion.h"
#include "vqf/vqf.hpp"

#include <new>

/* Static storage, placement-new: no heap on target. */
alignas(VQF) static unsigned char vqf_mem[sizeof(VQF)];
static VQF *filt;
static float g_sample_dt;

void htk_fusion_init(float sample_hz)
{
    g_sample_dt = 1.0f / sample_hz;
    filt = new (vqf_mem) VQF(g_sample_dt);
}

void htk_fusion_update(const float gyr[3], const float acc[3], float q[4])
{
    vqf_real_t g[3] = { gyr[0], gyr[1], gyr[2] };
    vqf_real_t a[3] = { acc[0], acc[1], acc[2] };

    filt->updateGyr(g);
    filt->updateAcc(a);

    vqf_real_t quat[4];
    filt->getQuat6D(quat); /* returns w, x, y, z */
    q[0] = (float)quat[0];
    q[1] = (float)quat[1];
    q[2] = (float)quat[2];
    q[3] = (float)quat[3];
}

bool htk_fusion_bias_ok(void)
{
    vqf_real_t bias[3];
    vqf_real_t sigma = filt->getBiasEstimate(bias);

    /* Converged when the estimate's uncertainty drops below 0.5 deg/s. */
    return sigma < (vqf_real_t)(0.5 * 3.14159265358979 / 180.0);
}

void htk_fusion_reset(void)
{
    filt->resetState();
}
