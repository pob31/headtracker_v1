/* Coverage-guided fuzz target (libFuzzer / MSVC /fsanitize=fuzzer).
 *
 * Build:  cmake -B build-fuzz -DHTK_FUZZ=ON [-DCMAKE_CXX_COMPILER=clang++]
 * Run:    htk_fuzz_parser corpus/ -max_total_time=300
 * Seed the corpus with the PROTOCOL.md vectors for faster convergence.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "headtracker/parser.hpp"

#include <cstddef>
#include <cstdint>

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size)
{
    /* Persistent parser: state carries across inputs, letting the fuzzer
     * explore cross-frame state (partial blocks, discard mode). A fresh one
     * each run would only ever fuzz the empty-state transitions. */
    static htk::Parser persistent;
    static uint64_t sink;

    htk::Parser fresh;
    for (htk::Parser *p : { &persistent, &fresh }) {
        p->on_orient = [](const htk_orient &o) { sink += o.seq; };
        p->on_status = [](const htk_status &s) { sink += s.rx_rate; };
        p->on_tracker_stat = [](const htk_tracker_stat &t) { sink += t.rate; };
        p->on_raw = [](const htk_raw &r) { sink += static_cast<uint64_t>(r.seq); };
        p->on_hello_resp = [](const htk_hello_resp &h) { sink += h.caps; };
        p->on_log = [](std::string_view s) { sink += s.size(); };
        p->feed(data, size);
    }
    return 0;
}
