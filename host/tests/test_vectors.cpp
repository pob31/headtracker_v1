/* Byte-exact conformance against the PROTOCOL.md normative vectors,
 * both directions: command encoding and stream decoding.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#include "doctest.h"
#include "vectors.hpp"

#include "headtracker/commands.hpp"
#include "headtracker/parser.hpp"

extern "C" {
#include "htk_crc16.h"
}

TEST_CASE("CRC-16/CCITT-FALSE check value")
{
    const uint8_t s[] = "123456789";
    CHECK(htk_crc16(s, 9) == HTK_CRC16_CHECK);
}

TEST_CASE("command encoders reproduce the spec frames byte-exactly")
{
    CHECK(htk::encode_reset_fusion(0x1234) == hexbytes(kFrameResetFusion));
    CHECK(htk::encode_set_mode(0x1234, HTK_MODE_RAW) == hexbytes(kFrameSetModeRaw));
    CHECK(htk::encode_identify(0x1234) == hexbytes(kFrameIdentify));
    CHECK(htk::encode_hello() == hexbytes(kFrameHello));
}

TEST_CASE("sensor-directed commands reject reserved target ids")
{
    CHECK(htk::encode_reset_fusion(HTK_ID_INVALID).empty());
    CHECK(htk::encode_reset_fusion(HTK_ID_SIM).empty());
    CHECK(htk::encode_identify(0x0000).empty());
    CHECK(htk::encode_set_rate(0xFFFF, 208).empty());
    CHECK(htk::encode_set_mode(0x0000, HTK_MODE_QUAT).empty());
    CHECK(htk::encode_set_mode(0x1234, 3).empty()); /* mode out of range */
}

TEST_CASE("ORIENT vector decodes to the spec's field values")
{
    htk::Parser p;
    htk_orient got{};
    int calls = 0;
    p.on_orient = [&](const htk_orient &o) { got = o; calls++; };

    const auto f = hexbytes(kFrameOrient);
    p.feed(f.data(), f.size());

    REQUIRE(calls == 1);
    CHECK(got.type == HTK_PKT_ORIENT);
    CHECK(got.id == 0x1234);
    CHECK(got.seq == 42);
    CHECK(got.t_us == 1000000u);
    CHECK(got.q_w == doctest::Approx(1.0f));
    CHECK(got.q_x == doctest::Approx(0.0f));
    CHECK(got.q_y == doctest::Approx(0.0f));
    CHECK(got.q_z == doctest::Approx(0.0f));
    CHECK(got.flags == HTK_ORIENT_BIAS_OK);
    CHECK(p.stats().frames_ok == 1);
}

TEST_CASE("TRACKER_STAT vector decodes to the spec's field values")
{
    htk::Parser p;
    htk_tracker_stat got{};
    p.on_tracker_stat = [&](const htk_tracker_stat &t) { got = t; };

    const auto f = hexbytes(kFrameTrackerStat);
    p.feed(f.data(), f.size());

    CHECK(got.id == 0x1234);
    CHECK(got.age_ms == 5u);
    CHECK(got.rate == 208);
    CHECK(got.seq_lost == 3u);
    CHECK(got.vbat_mV == 4012);
    CHECK(got.fw_major == 0);
    CHECK(got.fw_minor == 1);
    CHECK(got.fw_patch == 0);
    CHECK(got.mode == HTK_MODE_QUAT);
    CHECK(got.flags == HTK_TSTAT_LINK_UP);
}

TEST_CASE("HELLO_RESP vector decodes and passes the compatibility check")
{
    htk::Parser p;
    htk_hello_resp got{};
    p.on_hello_resp = [&](const htk_hello_resp &h) { got = h; };

    const auto f = hexbytes(kFrameHelloResp);
    p.feed(f.data(), f.size());

    CHECK(got.proto_ver == 1);
    CHECK(got.device == 1);
    CHECK(got.caps == (HTK_CAP_QUAT | HTK_CAP_RAW | HTK_CAP_SIM | HTK_CAP_MULTI));
    CHECK(htk::compatible(got));

    htk_hello_resp future = got;
    future.proto_ver = 2;
    CHECK_FALSE(htk::compatible(future));
}
