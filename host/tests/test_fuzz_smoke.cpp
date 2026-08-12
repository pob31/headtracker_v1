/* Deterministic fuzz smoke test — always runs in ctest.
 *
 * The guarantee under test (user requirement): garbage transmission can never
 * crash a consumer app. Random noise and structure-aware mutations of valid
 * frames are pushed through a persistent parser in random chunk sizes; the
 * invariants are "no crash / no hang / no false-size accept / counters add up".
 *
 * For deeper coverage-guided exploration build -DHTK_FUZZ=ON (fuzz/fuzz_parser.cpp).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"
#include "vectors.hpp"

#include "headtracker/parser.hpp"

namespace {

/* xorshift64* — deterministic across platforms, no <random> variation */
struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed ? seed : 0x9E3779B97F4A7C15ull) {}
    uint64_t next()
    {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return s * 0x2545F4914F6CDD1Dull;
    }
    uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

struct Harness {
    htk::Parser p;
    uint64_t accepted = 0;

    Harness()
    {
        /* Bind every callback: dispatch itself is part of the surface. The
         * size checks inside the callbacks are the "no false accept" oracle. */
        p.on_orient = [this](const htk_orient &o) {
            accepted++;
            REQUIRE(o.type == HTK_PKT_ORIENT);
        };
        p.on_status = [this](const htk_status &s) {
            accepted++;
            REQUIRE(s.type == HTK_PKT_STATUS);
        };
        p.on_tracker_stat = [this](const htk_tracker_stat &t) {
            accepted++;
            REQUIRE(t.type == HTK_PKT_TRACKER_STAT);
        };
        p.on_raw = [this](const htk_raw &r) {
            accepted++;
            REQUIRE(r.type == HTK_PKT_RAW);
        };
        p.on_hello_resp = [this](const htk_hello_resp &h) {
            accepted++;
            REQUIRE(h.type == HTK_PKT_HELLO_RESP);
        };
        p.on_log = [this](std::string_view s) {
            accepted++;
            REQUIRE(s.size() < HTK_MAX_PAYLOAD);
        };
    }

    void feed_chunked(Rng &rng, const std::vector<uint8_t> &data)
    {
        size_t i = 0;
        while (i < data.size()) {
            const size_t n = 1 + rng.below(97);
            const size_t take = (i + n > data.size()) ? data.size() - i : n;
            p.feed(data.data() + i, take);
            i += take;
        }
    }
};

} // namespace

TEST_CASE("fuzz: pure random noise, 2 MB, persistent parser")
{
    Rng rng(0xC0FFEE);
    Harness h;
    std::vector<uint8_t> buf(4096);
    for (int round = 0; round < 512; round++) {
        for (auto &b : buf) {
            b = static_cast<uint8_t>(rng.next());
        }
        h.feed_chunked(rng, buf);
    }
    /* random noise essentially never forms a CRC-valid frame */
    CHECK(h.p.stats().frames_ok == h.accepted);
}

TEST_CASE("fuzz: structure-aware mutations of valid frames")
{
    Rng rng(0xDEADBEEF);
    Harness h;
    const auto seeds = all_valid_frames();

    for (int iter = 0; iter < 40000; iter++) {
        auto f = seeds[rng.below(static_cast<uint32_t>(seeds.size()))];

        switch (rng.below(6)) {
        case 0: /* flip random byte */
            f[rng.below(static_cast<uint32_t>(f.size()))] ^=
                static_cast<uint8_t>(1 + rng.below(255));
            break;
        case 1: /* truncate */
            f.resize(1 + rng.below(static_cast<uint32_t>(f.size())));
            break;
        case 2: /* insert random byte */
            f.insert(f.begin() + rng.below(static_cast<uint32_t>(f.size())),
                     static_cast<uint8_t>(rng.next()));
            break;
        case 3: /* delete a byte */
            f.erase(f.begin() + rng.below(static_cast<uint32_t>(f.size())));
            break;
        case 4: /* splice two frames mid-way */
        {
            const auto &g = seeds[rng.below(static_cast<uint32_t>(seeds.size()))];
            const size_t cut = rng.below(static_cast<uint32_t>(f.size()));
            f.resize(cut);
            f.insert(f.end(), g.begin() + g.size() / 2, g.end());
            break;
        }
        case 5: /* inject stray delimiters */
            f.insert(f.begin() + rng.below(static_cast<uint32_t>(f.size())), 0x00);
            break;
        }
        h.feed_chunked(rng, f);

        /* occasionally interleave an intact frame: parser must still be in a
         * state where valid traffic gets through */
        if (iter % 64 == 0) {
            const uint8_t zero = 0;
            h.p.feed(&zero, 1); /* close any partial block */
            const uint64_t before = h.p.stats().frames_ok;
            const auto &good = seeds[5]; /* ORIENT */
            h.p.feed(good.data(), good.size());
            REQUIRE(h.p.stats().frames_ok == before + 1);
        }
    }

    const auto &st = h.p.stats();
    CHECK(st.frames_ok == h.accepted);
    /* mutations must actually have exercised the error paths */
    CHECK(st.decode_errors > 0);
    CHECK(st.frames_ok > 0);
    INFO("ok=" << st.frames_ok << " decode_err=" << st.decode_errors
                << " size_mismatch=" << st.size_mismatch
                << " unknown=" << st.unknown_types
                << " oversize=" << st.oversize_resyncs);
}

TEST_CASE("fuzz: valid stream chopped at every possible boundary pair")
{
    const auto f = hexbytes(kFrameOrient);
    for (size_t a = 1; a < f.size() - 1; a++) {
        for (size_t b = a; b < f.size(); b++) {
            Harness h;
            h.p.feed(f.data(), a);
            h.p.feed(f.data() + a, b - a);
            h.p.feed(f.data() + b, f.size() - b);
            REQUIRE(h.p.stats().frames_ok == 1);
        }
    }
}
