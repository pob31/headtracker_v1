/* Parser robustness: chunking, resync, corruption, forward compatibility.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"
#include "vectors.hpp"

#include "headtracker/parser.hpp"

extern "C" {
#include "htk_frame.h"
}

namespace {

struct Counting {
    htk::Parser p;
    int orients = 0;
    Counting()
    {
        p.on_orient = [this](const htk_orient &) { orients++; };
    }
};

} // namespace

TEST_CASE("one byte at a time delivery")
{
    Counting c;
    const auto f = hexbytes(kFrameOrient);
    for (uint8_t b : f) {
        c.p.feed(&b, 1);
    }
    CHECK(c.orients == 1);
    CHECK(c.p.stats().frames_ok == 1);
}

TEST_CASE("many frames in a single feed, all boundaries")
{
    Counting c;
    std::vector<uint8_t> stream;
    const auto f = hexbytes(kFrameOrient);
    for (int i = 0; i < 100; i++) {
        stream.insert(stream.end(), f.begin(), f.end());
    }
    c.p.feed(stream.data(), stream.size());
    CHECK(c.orients == 100);
}

TEST_CASE("mid-stream attach resynchronizes on the next delimiter")
{
    Counting c;
    const auto f = hexbytes(kFrameOrient);
    /* start in the middle of a frame: partial garbage, then two good frames */
    c.p.feed(f.data() + 7, f.size() - 7); /* tail of a frame incl. delimiter */
    c.p.feed(f.data(), f.size());
    c.p.feed(f.data(), f.size());
    CHECK(c.orients == 2);
    CHECK(c.p.stats().decode_errors == 1); /* the truncated tail */
}

TEST_CASE("every single-byte corruption is rejected without a crash")
{
    const auto f = hexbytes(kFrameOrient);
    for (size_t i = 0; i + 1 < f.size(); i++) { /* skip trailing delimiter */
        for (int delta : { 1, 0x80 }) {
            Counting c;
            auto bad = f;
            bad[i] = static_cast<uint8_t>(bad[i] ^ delta);
            c.p.feed(bad.data(), bad.size());
            /* corrupting a byte to 0x00 splits the frame; either way the
             * CRC-checked payload must never be accepted */
            CHECK(c.orients == 0);
            CHECK(c.p.stats().frames_ok == 0);
        }
    }
}

TEST_CASE("empty frames are ignored silently")
{
    Counting c;
    const std::vector<uint8_t> zeros(16, 0x00);
    c.p.feed(zeros.data(), zeros.size());
    CHECK(c.p.stats().frames_ok == 0);
    CHECK(c.p.stats().decode_errors == 0);

    const auto f = hexbytes(kFrameOrient);
    c.p.feed(f.data(), f.size());
    CHECK(c.orients == 1);
}

TEST_CASE("unknown packet type is skipped and counted, not an error")
{
    /* Forward compatibility: build a valid frame with unassigned type 0x60 */
    const uint8_t payload[] = { 0x60, 0xAA, 0xBB };
    uint8_t frame[HTK_MAX_FRAME];
    const size_t n = htk_frame_encode(payload, sizeof(payload), frame, sizeof(frame));
    REQUIRE(n > 0);

    Counting c;
    c.p.feed(frame, n);
    CHECK(c.p.stats().unknown_types == 1);
    CHECK(c.p.stats().decode_errors == 0);

    const auto f = hexbytes(kFrameOrient);
    c.p.feed(f.data(), f.size());
    CHECK(c.orients == 1); /* still in sync afterwards */
}

TEST_CASE("known type with wrong length counts as size mismatch")
{
    /* ORIENT type byte but a short body */
    const uint8_t payload[] = { HTK_PKT_ORIENT, 0x01, 0x02, 0x03 };
    uint8_t frame[HTK_MAX_FRAME];
    const size_t n = htk_frame_encode(payload, sizeof(payload), frame, sizeof(frame));
    REQUIRE(n > 0);

    Counting c;
    c.p.feed(frame, n);
    CHECK(c.orients == 0);
    CHECK(c.p.stats().size_mismatch == 1);
}

TEST_CASE("oversize garbage block triggers resync and later recovery")
{
    Counting c;
    std::vector<uint8_t> junk(300, 0x55); /* no delimiter for 300 bytes */
    c.p.feed(junk.data(), junk.size());
    const uint8_t zero = 0x00;
    c.p.feed(&zero, 1);
    CHECK(c.p.stats().oversize_resyncs == 1);

    const auto f = hexbytes(kFrameOrient);
    c.p.feed(f.data(), f.size());
    CHECK(c.orients == 1);
}

TEST_CASE("LOG frames deliver text of any length including empty")
{
    htk::Parser p;
    std::string got;
    int calls = 0;
    p.on_log = [&](std::string_view s) { got = std::string(s); calls++; };

    const uint8_t payload[] = { HTK_PKT_LOG, 'h', 'i' };
    uint8_t frame[HTK_MAX_FRAME];
    size_t n = htk_frame_encode(payload, sizeof(payload), frame, sizeof(frame));
    p.feed(frame, n);
    CHECK(calls == 1);
    CHECK(got == "hi");

    const uint8_t empty_log[] = { HTK_PKT_LOG };
    n = htk_frame_encode(empty_log, sizeof(empty_log), frame, sizeof(frame));
    p.feed(frame, n);
    CHECK(calls == 2);
    CHECK(got.empty());
}

TEST_CASE("reset drops partial state")
{
    Counting c;
    const auto f = hexbytes(kFrameOrient);
    c.p.feed(f.data(), 10); /* partial frame buffered */
    c.p.reset();
    c.p.feed(f.data(), f.size());
    CHECK(c.orients == 1);
    CHECK(c.p.stats().decode_errors == 0); /* no corpse from the partial */
}
