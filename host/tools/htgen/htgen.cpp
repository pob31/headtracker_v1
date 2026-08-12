/* htgen — synthetic dongle stream generator.
 *
 * Emits the exact byte stream a dongle would produce (PROTOCOL.md Part 1):
 * ORIENT sweeps per tracker, STATUS + TRACKER_STAT at 1 Hz cadence. Used to
 * develop and test consumers (and htmon later) with zero hardware, and as a
 * fuzz-corpus/garbage-robustness source via --garbage.
 *
 *   htgen [--frames N] [--rate HZ] [--trackers K] [--garbage PCT]
 *         [--seed S] [--out FILE]
 *
 * Output is binary on stdout unless --out is given.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

extern "C" {
#include "htk_frame.h"
#include "htk_protocol.h"
}

#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

namespace {

struct Rng {
    uint64_t s;
    explicit Rng(uint64_t seed) : s(seed ? seed : 1) {}
    uint64_t next()
    {
        s ^= s >> 12;
        s ^= s << 25;
        s ^= s >> 27;
        return s * 0x2545F4914F6CDD1Dull;
    }
    uint32_t below(uint32_t n) { return static_cast<uint32_t>(next() % n); }
};

struct Options {
    unsigned frames = 2080; /* ~10 s at 208 Hz */
    unsigned rate = 208;
    unsigned trackers = 1;
    unsigned garbage_pct = 0;
    uint64_t seed = 42;
    std::string out;
    bool hello = true;
};

void emit(FILE *f, Rng &rng, const Options &opt, const void *payload, size_t len)
{
    uint8_t frame[HTK_MAX_FRAME];
    const size_t n = htk_frame_encode(static_cast<const uint8_t *>(payload), len,
                                      frame, sizeof(frame));
    if (n == 0) {
        return;
    }
    if (opt.garbage_pct > 0 && rng.below(100) < opt.garbage_pct) {
        /* corrupt this frame: byte flip, truncation, or leading noise */
        switch (rng.below(3)) {
        case 0:
            frame[rng.below(static_cast<uint32_t>(n))] ^=
                static_cast<uint8_t>(1 + rng.below(255));
            fwrite(frame, 1, n, f);
            break;
        case 1:
            fwrite(frame, 1, 1 + rng.below(static_cast<uint32_t>(n - 1)), f);
            break;
        default: {
            uint8_t junk[16];
            for (auto &b : junk) {
                b = static_cast<uint8_t>(rng.next());
            }
            fwrite(junk, 1, sizeof(junk), f);
            fwrite(frame, 1, n, f);
            break;
        }
        }
        return;
    }
    fwrite(frame, 1, n, f);
}

} // namespace

int main(int argc, char **argv)
{
    Options opt;
    for (int i = 1; i < argc; i++) {
        const std::string a = argv[i];
        auto val = [&]() -> const char * { return (i + 1 < argc) ? argv[++i] : "0"; };
        if (a == "--frames") {
            opt.frames = std::stoul(val());
        } else if (a == "--rate") {
            opt.rate = std::stoul(val());
        } else if (a == "--trackers") {
            opt.trackers = std::stoul(val());
        } else if (a == "--garbage") {
            opt.garbage_pct = std::stoul(val());
        } else if (a == "--seed") {
            opt.seed = std::stoull(val());
        } else if (a == "--out") {
            opt.out = val();
        } else if (a == "--no-hello") {
            opt.hello = false;
        } else {
            std::fprintf(stderr,
                         "usage: htgen [--frames N] [--rate HZ] [--trackers K]"
                         " [--garbage PCT] [--seed S] [--out FILE] [--no-hello]\n");
            return 2;
        }
    }
    if (opt.rate == 0 || opt.trackers == 0 || opt.trackers > 8) {
        std::fprintf(stderr, "htgen: rate must be > 0, trackers 1..8\n");
        return 2;
    }

    FILE *f = stdout;
    if (!opt.out.empty()) {
        f = std::fopen(opt.out.c_str(), "wb");
        if (!f) {
            std::perror("htgen: fopen");
            return 1;
        }
    }
#ifdef _WIN32
    else {
        _setmode(_fileno(stdout), _O_BINARY);
    }
#endif

    Rng rng(opt.seed);
    const uint32_t period_us = 1000000u / opt.rate;
    uint16_t seq[8] = { 0 };

    /* A capture is replayed into a client that opens the session by sending
     * HELLO (PROTOCOL.md §1.7) and refuses to stream until it is answered.
     * A one-way file cannot answer anything, so the answer leads the stream —
     * close enough for replay, and it makes htgen output a complete session
     * rather than a fragment. --no-hello reproduces a mid-stream attach, where
     * the consumer joins a dongle that is already running. */
    if (opt.hello) {
        htk_hello_resp hr;
        std::memset(&hr, 0, sizeof(hr));
        hr.type = HTK_PKT_HELLO_RESP;
        hr.proto_ver = HTK_PROTO_VERSION;
        hr.fw_major = HTK_FW_MAJOR;
        hr.fw_minor = HTK_FW_MINOR;
        hr.fw_patch = HTK_FW_PATCH;
        hr.device = 1;
        hr.caps = HTK_CAP_QUAT | HTK_CAP_RAW | HTK_CAP_SIM | HTK_CAP_MULTI;
        /* Never corrupted: garbage tolerance is about the data path, and a
         * capture whose handshake is mangled would just fail to open. */
        uint8_t frame[HTK_MAX_FRAME];
        const size_t n = htk_frame_encode(reinterpret_cast<const uint8_t *>(&hr),
                                          sizeof(hr), frame, sizeof(frame));
        fwrite(frame, 1, n, f);
    }

    for (unsigned n = 0; n < opt.frames; n++) {
        const uint32_t t_us = n * period_us;

        for (unsigned k = 0; k < opt.trackers; k++) {
            /* 10 deg/s yaw sweep, phase-offset per tracker */
            const float a = 10.0f * 3.14159265f / 180.0f * (t_us * 1e-6f) +
                            k * 0.5f;
            htk_orient o;
            std::memset(&o, 0, sizeof(o));
            o.type = HTK_PKT_ORIENT;
            o.id = static_cast<uint16_t>(0x1000 + k);
            o.seq = seq[k]++;
            o.t_us = t_us;
            o.q_w = std::cos(a / 2.0f);
            o.q_z = std::sin(a / 2.0f);
            o.flags = HTK_ORIENT_BIAS_OK | HTK_ORIENT_SIM;
            emit(f, rng, opt, &o, sizeof(o));
        }

        if (n % opt.rate == 0) { /* 1 Hz housekeeping */
            htk_status st;
            std::memset(&st, 0, sizeof(st));
            st.type = HTK_PKT_STATUS;
            st.uptime_ms = t_us / 1000u;
            st.rx_rate = static_cast<uint16_t>(opt.rate * opt.trackers);
            st.n_trackers = static_cast<uint8_t>(opt.trackers);
            emit(f, rng, opt, &st, sizeof(st));

            for (unsigned k = 0; k < opt.trackers; k++) {
                htk_tracker_stat ts;
                std::memset(&ts, 0, sizeof(ts));
                ts.type = HTK_PKT_TRACKER_STAT;
                ts.id = static_cast<uint16_t>(0x1000 + k);
                ts.age_ms = 0;
                ts.rate = static_cast<uint16_t>(opt.rate);
                ts.vbat_mV = 4100;
                ts.fw_major = HTK_FW_MAJOR;
                ts.fw_minor = HTK_FW_MINOR;
                ts.fw_patch = HTK_FW_PATCH;
                ts.mode = HTK_MODE_QUAT;
                ts.flags = HTK_TSTAT_LINK_UP;
                emit(f, rng, opt, &ts, sizeof(ts));
            }
        }
    }

    if (f != stdout) {
        std::fclose(f);
    }
    return 0;
}
