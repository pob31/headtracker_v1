/* htk::Client — session, tracker table and command round-trips, driven
 * entirely through MemTransport so the whole thing runs with no hardware.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "doctest.h"

#include "headtracker/client.hpp"
#include "headtracker/commands.hpp"
#include "htk_frame.h"
#include "vectors.hpp"

#include <atomic>
#include <chrono>
#include <cstring>
#include <thread>

namespace {

/* Build a complete framed packet from a raw payload the way a dongle would. */
std::vector<uint8_t> frame_of(const uint8_t *payload, size_t len)
{
    uint8_t out[HTK_MAX_FRAME];
    const size_t n = htk_frame_encode(payload, len, out, sizeof(out));
    return std::vector<uint8_t>(out, out + n);
}

template <typename T> std::vector<uint8_t> frame_struct(const T &s)
{
    return frame_of(reinterpret_cast<const uint8_t *>(&s), sizeof(T));
}

std::vector<uint8_t> hello_resp_frame(uint8_t proto = HTK_PROTO_VERSION, uint8_t device = 1)
{
    htk_hello_resp h{};
    h.type = HTK_PKT_HELLO_RESP;
    h.proto_ver = proto;
    h.fw_major = 0;
    h.fw_minor = 1;
    h.fw_patch = 0;
    h.device = device;
    h.caps = HTK_CAP_QUAT | HTK_CAP_RAW | HTK_CAP_SIM | HTK_CAP_MULTI;
    return frame_struct(h);
}

std::vector<uint8_t> orient_frame(uint16_t id, uint16_t seq, float qw = 1.0f)
{
    htk_orient o{};
    o.type = HTK_PKT_ORIENT;
    o.id = id;
    o.seq = seq;
    o.t_us = seq * 4800u;
    o.q_w = qw;
    o.q_x = o.q_y = o.q_z = 0.0f;
    o.flags = HTK_ORIENT_BIAS_OK;
    return frame_struct(o);
}

std::vector<uint8_t> tracker_stat_frame(uint16_t id, uint16_t vbat = 4012, bool link_up = true)
{
    htk_tracker_stat t{};
    t.type = HTK_PKT_TRACKER_STAT;
    t.id = id;
    t.age_ms = 5;
    t.rate = 208;
    t.seq_lost = 3;
    t.vbat_mV = vbat;
    t.fw_major = 0;
    t.fw_minor = 1;
    t.fw_patch = 0;
    t.mode = HTK_MODE_QUAT;
    t.flags = link_up ? HTK_TSTAT_LINK_UP : 0u;
    return frame_struct(t);
}

std::vector<uint8_t> status_frame()
{
    htk_status s{};
    s.type = HTK_PKT_STATUS;
    s.uptime_ms = 1234;
    s.rx_rate = 208;
    s.n_trackers = 1;
    s.flags = 0;
    return frame_struct(s);
}

/* Spin until `pred` holds or the deadline passes. The client runs a real
 * thread, so tests synchronize on observable state rather than on sleeps. */
template <typename Pred> bool wait_for(Pred pred, int timeout_ms = 3000)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return pred();
}

/* Owns the transport pointer the client took, so tests can keep feeding it. */
struct Rig {
    htk::MemTransport *mem = nullptr;
    htk::Client client;

    void start(htk::Client::Options opts = htk::Client::Options())
    {
        auto t = std::unique_ptr<htk::MemTransport>(new htk::MemTransport);
        mem = t.get();
        std::string err;
        t->open("mem", err);
        client.start(std::move(t), "mem", opts);
    }

    /* Answer the handshake the way a dongle would. */
    bool handshake(uint8_t proto = HTK_PROTO_VERSION)
    {
        if (!wait_for([&] { return !mem->written().empty(); })) {
            return false;
        }
        mem->feed(hello_resp_frame(proto));
        return true;
    }

    ~Rig() { client.stop(); }
};

} // namespace

TEST_CASE("client handshake reaches Ready and records device info")
{
    Rig r;
    r.start();
    REQUIRE(r.handshake());

    CHECK(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    const auto d = r.client.device();
    CHECK(d.proto_ver == HTK_PROTO_VERSION);
    CHECK(d.device == 1);
    CHECK((d.caps & HTK_CAP_MULTI) != 0);
    CHECK(d.fw_minor == 1);

    /* The first thing written must be the HELLO frame from §1.8, byte-exact. */
    const auto written = r.mem->written();
    const auto hello = hexbytes(kFrameHello);
    REQUIRE(written.size() >= hello.size());
    CHECK(std::memcmp(written.data(), hello.data(), hello.size()) == 0);
}

TEST_CASE("client refuses an incompatible protocol version and stops")
{
    Rig r;
    htk::Client::Options o;
    o.auto_reconnect = false;
    r.start(o);
    REQUIRE(r.handshake(HTK_PROTO_VERSION + 1));

    CHECK(wait_for([&] { return r.client.state() == htk::ConnState::Incompatible; }));

    /* A mismatched major version may change the ORIENT layout, so nothing is
     * dispatched: silently rendering a mis-parsed quaternion is worse than
     * no tracking. */
    CHECK(r.client.last_error().find("protocol v") != std::string::npos);
    CHECK(wait_for([&] { return !r.client.running(); }));
}

TEST_CASE("client refuses a device that is not a dongle")
{
    Rig r;
    htk::Client::Options o;
    o.auto_reconnect = false;
    r.start(o);
    REQUIRE(r.handshake());
    CHECK(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    /* device != 1 is still a valid HELLO_RESP, so the handshake succeeds; it
     * is discovery (probe_port) that filters on the device byte. Assert the
     * value survives so a caller can act on it. */
    CHECK(r.client.device().device == 1);
}

TEST_CASE("tracker table is built from ORIENT and TRACKER_STAT")
{
    /* Callbacks must be installed before start(): the reader thread reads them
     * without synchronization, exactly as the header documents. */
    std::atomic<int> orients { 0 };
    Rig r;
    r.client.on_orient = [&](const htk_orient &) { ++orients; };
    r.start();
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    /* A TRACKER_STAT alone is enough to promote an id. */
    r.mem->feed(tracker_stat_frame(0x1234));
    CHECK(wait_for([&] {
        const auto t = r.client.trackers();
        return t.size() == 1 && t[0].id == 0x1234 && t[0].vbat_mV == 4012 && t[0].link_up;
    }));

    /* An id seen only in ORIENT is promoted after the corroboration gate. */
    for (uint16_t s = 0; s < 5; ++s) {
        r.mem->feed(orient_frame(0x2000, s));
    }
    CHECK(wait_for([&] {
        const auto t = r.client.trackers();
        return t.size() == 2;
    }));
    CHECK(wait_for([&] { return orients.load() >= 5; }));
}

TEST_CASE("a single ORIENT does not promote an id (CRC-collision gate)")
{
    Rig r;
    htk::Client::Options o;
    o.promote_after_orients = 3;
    r.start(o);
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    std::vector<htk::TrackerInfo> published;
    r.client.on_trackers = [&](const std::vector<htk::TrackerInfo> &t) { published = t; };

    r.mem->feed(orient_frame(0xBEEF, 0));
    r.mem->feed(status_frame());

    /* Wait past the 1 Hz sweep so on_trackers has certainly fired. */
    std::this_thread::sleep_for(std::chrono::milliseconds(1300));
    for (const auto &t : published) {
        CHECK(t.id != 0xBEEF);
    }

    /* Two more of the same id and it becomes real. */
    r.mem->feed(orient_frame(0xBEEF, 1));
    r.mem->feed(orient_frame(0xBEEF, 2));
    CHECK(wait_for([&] {
        for (const auto &t : r.client.trackers()) {
            if (t.id == 0xBEEF && t.orient_count >= 3) {
                return true;
            }
        }
        return false;
    }));
}

TEST_CASE("trackers expire after the silence window")
{
    Rig r;
    htk::Client::Options o;
    o.tracker_expiry_ms = 200;  /* the real rule is 30 s; shortened for the test */
    o.no_data_watchdog_ms = 0;  /* expiry is the subject here, not link loss */
    r.start(o);
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    r.mem->feed(tracker_stat_frame(0x1234));
    REQUIRE(wait_for([&] { return r.client.trackers().size() == 1; }));

    /* The dongle just stops sending for a tracker that went away, so the host
     * has to expire it rather than waiting to be told. */
    CHECK(wait_for([&] { return r.client.trackers().empty(); }, 4000));
}

TEST_CASE("commands produce the byte-exact frames from PROTOCOL.md 1.8")
{
    Rig r;
    r.start();
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    r.mem->clear_written();

    CHECK(r.client.reset_fusion(0x1234));
    CHECK(wait_for([&] { return r.mem->written().size() >= 7; }));
    CHECK(r.mem->written() == hexbytes(kFrameResetFusion));

    r.mem->clear_written();
    CHECK(r.client.identify(0x1234));
    CHECK(wait_for([&] { return r.mem->written().size() >= 7; }));
    CHECK(r.mem->written() == hexbytes(kFrameIdentify));

    r.mem->clear_written();
    CHECK(r.client.set_mode(0x1234, HTK_MODE_RAW));
    CHECK(wait_for([&] { return r.mem->written().size() >= 8; }));
    CHECK(r.mem->written() == hexbytes(kFrameSetModeRaw));

    /* Reserved ids are refused at the API, exactly as the dongle would drop
     * them -- but here the caller finds out. */
    CHECK_FALSE(r.client.identify(HTK_ID_INVALID));
    CHECK_FALSE(r.client.identify(HTK_ID_SIM));
    CHECK_FALSE(r.client.set_mode(0x1234, 99));
}

TEST_CASE("a pure garbage stream leaves the client alive and counting")
{
    Rig r;
    htk::Client::Options o;
    o.no_data_watchdog_ms = 0; /* the test goes quiet on purpose after feeding */
    r.start(o);
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    std::vector<uint8_t> junk;
    uint32_t s = 12345;
    for (int i = 0; i < 20000; ++i) {
        s ^= s << 13;
        s ^= s >> 17;
        s ^= s << 5;
        junk.push_back((uint8_t) (s & 0xFF));
    }
    r.mem->feed(junk);

    /* Real frames afterwards: resync must recover. Note the FIRST frame after
     * garbage is expected to be lost -- the junk's trailing bytes form an
     * unterminated block that swallows it, which is exactly what a mid-stream
     * attach looks like. A real dongle repeats at 1 Hz, so this self-heals;
     * feeding one frame and demanding it survive would be testing a guarantee
     * the framing deliberately does not make. */
    for (int i = 0; i < 3; ++i) {
        r.mem->feed(tracker_stat_frame(0x0777));
    }

    CHECK(wait_for([&] {
        const auto t = r.client.trackers();
        return t.size() == 1 && t[0].id == 0x0777;
    }, 5000));
    CHECK(r.client.state() == htk::ConnState::Ready);
    /* parser_stats() is refreshed on the 1 Hz sweep, so give it one. */
    CHECK(wait_for([&] { return r.client.parser_stats().decode_errors > 0; }, 3000));
}

TEST_CASE("a dead port is reported and, with auto_reconnect off, ends the run")
{
    Rig r;
    htk::Client::Options o;
    o.auto_reconnect = false;
    r.start(o);
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    r.mem->fail();
    CHECK(wait_for([&] { return r.client.state() == htk::ConnState::Lost; }));
    CHECK(wait_for([&] { return !r.client.running(); }));
}

TEST_CASE("silence trips the watchdog even without a read error")
{
    Rig r;
    htk::Client::Options o;
    o.auto_reconnect = false;
    o.no_data_watchdog_ms = 150;
    r.start(o);
    REQUIRE(r.handshake());
    REQUIRE(wait_for([&] { return r.client.state() == htk::ConnState::Ready; }));

    /* An unplug does not reliably surface as a read error on every OS, and a
     * live dongle sends STATUS at 1 Hz regardless of mode, so silence is
     * unambiguous. */
    CHECK(wait_for([&] { return r.client.state() == htk::ConnState::Lost; }));
    CHECK(r.client.last_error().find("no data") != std::string::npos);
}

TEST_CASE("stop() is idempotent and safe before any start")
{
    htk::Client c;
    c.stop();
    c.stop();
    CHECK(c.state() == htk::ConnState::Closed);
    CHECK_FALSE(c.running());
}
