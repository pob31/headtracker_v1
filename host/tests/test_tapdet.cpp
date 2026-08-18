/* Software double-tap detector: synthetic waveforms through the same C code
 * the head unit runs.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"

extern "C" {
#include "htk_tapdet.h"
}

#include <cmath>
#include <vector>

namespace {

constexpr float kOdr = 416.0f;
constexpr uint32_t kDtUs = 2404;

struct Feeder {
    htk_tapdet d {};
    uint32_t t = 0;
    int events = 0;

    Feeder() { htk_tapdet_init(&d, kOdr, nullptr); }

    /* level gravity + optional transient */
    void samples(int n, float spike = 0.0f, int spike_len = 1)
    {
        for (int i = 0; i < n; i++) {
            const float dev = (i < spike_len) ? spike : 0.0f;
            const float acc[3] = { 0.0f, 0.0f, 9.81f + dev };
            if (htk_tapdet_feed(&d, t, acc)) {
                events++;
            }
            t += kDtUs;
        }
    }

    void quiet_ms(int ms) { samples((int) (ms * kOdr / 1000.0f)); }
    void tap() { samples(4, 25.0f, 3); } /* ~7 ms, 25 m/s^2 transient */
};

} // namespace

TEST_CASE("double tap within the window fires exactly once")
{
    Feeder f;
    f.quiet_ms(500); /* baseline warm-up */
    f.tap();
    f.quiet_ms(150);
    f.tap();
    f.quiet_ms(300);
    CHECK(f.events == 1);
}

TEST_CASE("single tap never fires")
{
    Feeder f;
    f.quiet_ms(500);
    f.tap();
    f.quiet_ms(2000);
    CHECK(f.events == 0);
}

TEST_CASE("two taps too far apart are two singles, not a double")
{
    Feeder f;
    f.quiet_ms(500);
    f.tap();
    f.quiet_ms(700); /* > gap_max 500 ms */
    f.tap();
    f.quiet_ms(700);
    CHECK(f.events == 0);
}

TEST_CASE("ringing right after a tap does not count as the second tap")
{
    Feeder f;
    f.quiet_ms(500);
    f.tap();
    f.quiet_ms(30); /* < gap_min 70 ms: mechanical ringing */
    f.tap();
    f.quiet_ms(2000);
    CHECK(f.events == 0);
}

TEST_CASE("sustained shaking is handling, not taps")
{
    Feeder f;
    f.quiet_ms(500);
    /* 200 ms of continuous large excitation */
    f.samples((int) (0.2f * kOdr), 25.0f, (int) (0.2f * kOdr));
    f.quiet_ms(150);
    f.tap(); /* a lone tap after the shake: still no double */
    f.quiet_ms(2000);
    CHECK(f.events == 0);
}

TEST_CASE("refractory: a triple tap is one event, and re-arms afterwards")
{
    Feeder f;
    f.quiet_ms(500);
    f.tap();
    f.quiet_ms(150);
    f.tap();
    f.quiet_ms(150);
    f.tap(); /* inside refractory */
    f.quiet_ms(1500);
    CHECK(f.events == 1);

    f.tap();
    f.quiet_ms(150);
    f.tap();
    f.quiet_ms(300);
    CHECK(f.events == 2);
}

TEST_CASE("gentle head motion never triggers")
{
    htk_tapdet d;
    htk_tapdet_init(&d, kOdr, nullptr);
    uint32_t t = 0;
    int events = 0;

    /* 10 s of nodding: gravity vector swinging ±20 deg at 2 Hz — large
     * orientation change, small |acc| modulation */
    for (int i = 0; i < (int) (10 * kOdr); i++) {
        const float ang = 0.35f * std::sin(2.0f * 3.14159f * 2.0f * t * 1e-6f);
        const float acc[3] = { 9.81f * std::sin(ang), 0.0f,
                               9.81f * std::cos(ang) };
        if (htk_tapdet_feed(&d, t, acc)) {
            events++;
        }
        t += kDtUs;
    }
    CHECK(events == 0);
}
