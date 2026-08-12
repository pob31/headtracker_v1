/* htk::Client — the dongle session (PROTOCOL.md §1.7).
 *
 * Owns a Transport, a reader thread and a Parser, and turns the byte stream
 * into the two things an application actually wants: a live tracker list to
 * choose from, and orientation samples for the chosen one.
 *
 * Implements the consumer obligations of §1.9 on the application's behalf:
 * HELLO handshake with a version check, unknown packet types skipped, flag
 * fields masked, malformed frames counted and resynchronized.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include "parser.hpp"
#include "transport.hpp"

namespace htk {

enum class ConnState {
    Closed,       /* not running */
    Opening,      /* looking for / opening a port */
    Handshaking,  /* port open, waiting for HELLO_RESP */
    Ready,        /* handshake done, streaming */
    Incompatible, /* dongle speaks a different protocol major version */
    Lost,         /* port died; auto_reconnect decides what happens next */
};

const char *to_string(ConnState s);

struct DeviceInfo {
    uint8_t proto_ver = 0;
    uint8_t fw_major = 0, fw_minor = 0, fw_patch = 0;
    uint8_t device = 0; /* 1 = dongle */
    uint16_t caps = 0;  /* HTK_CAP_*, already masked to known bits */
    std::string port;
};

struct TrackerInfo {
    uint16_t id = 0;
    uint32_t age_ms = 0;   /* as reported by the dongle */
    uint16_t rate = 0;     /* packets/s seen by the dongle */
    uint32_t seq_lost = 0; /* cumulative, as seen by the dongle */
    uint16_t vbat_mV = 0;  /* 0 = unknown */
    uint8_t fw_major = 0, fw_minor = 0, fw_patch = 0;
    uint8_t mode = 0; /* HTK_MODE_* */
    bool link_up = false;
    bool sim = false;

    /* Host-side bookkeeping, in this host's monotonic clock. */
    uint64_t last_seen_ms = 0;
    uint32_t orient_count = 0;
};

class Client {
public:
    struct Options {
        bool auto_reconnect = true;
        int reconnect_ms = 1000;
        int reconnect_max_ms = 5000;
        int read_timeout_ms = 20;

        /* An unplug does not reliably surface as a read error on every OS, so
         * silence is also treated as death. */
        int no_data_watchdog_ms = 2000;

        /* Mirror of the dongle's own 30 s rule: it simply stops sending
         * TRACKER_STAT for a tracker, so the host must expire it. */
        int tracker_expiry_ms = 30000;

        /* An id is only reported as a tracker once it has been corroborated:
         * either a TRACKER_STAT, or this many ORIENT frames. A CRC-16
         * collision is ~1/65536 per corrupted frame, which over a long show on
         * a noisy link is roughly one bogus id -- without a gate it would sit
         * in the user's tracker list for the rest of the session. */
        uint32_t promote_after_orients = 3;
    };

    Client() = default;
    ~Client();

    Client(const Client &) = delete;
    Client &operator=(const Client &) = delete;

    /* --- Callbacks. Set BEFORE start(). ---
     *
     * ALL of these fire on the reader thread, and none of them may block.
     * That is deliberate: on_orient is the latency-critical path (208 Hz per
     * tracker) and marshalling it through a queue would spend part of the very
     * budget this product exists to protect. The intended consumer shape is to
     * convert and drop the result into a lock-free snapshot for the render
     * thread. Anything needing a GUI thread marshals with its own framework;
     * keeping that out of here is what lets this library stay framework-free.
     *
     * Consumers that would rather poll can ignore these and call state(),
     * device() and trackers() instead. */
    std::function<void(const htk_orient &)> on_orient;
    std::function<void(const std::vector<TrackerInfo> &)> on_trackers;
    std::function<void(ConnState, const std::string &)> on_state;
    std::function<void(std::string_view)> on_log;

    /* Starts the reader thread. `target` is a port path, or empty to
     * auto-discover. Returns false only if a thread is already running. */
    bool start(std::unique_ptr<Transport> transport, std::string target, Options opts = {});

    /* Convenience: a real serial port, auto-discovered when target is empty. */
    bool start(std::string target = {}, Options opts = {});

    /* Signals the reader thread and joins it. Safe to call twice, and safe to
     * call from a callback's thread only AFTER returning from the callback.
     * Must complete before anything the callbacks touch is destroyed. */
    void stop();

    bool running() const { return running_.load(std::memory_order_acquire); }
    ConnState state() const { return state_.load(std::memory_order_acquire); }
    std::string last_error() const;

    DeviceInfo device() const;
    std::vector<TrackerInfo> trackers() const;
    Parser::Stats parser_stats() const;

    /* --- Commands (PROTOCOL.md §1.5). Callable from any thread. ---
     * Each returns false if the frame could not be encoded (a reserved tracker
     * id, an out-of-range mode) or could not be written. */
    bool identify(uint16_t tracker_id);
    bool reset_fusion(uint16_t tracker_id);
    bool set_rate(uint16_t tracker_id, uint16_t hz);
    bool set_mode(uint16_t tracker_id, uint8_t mode);
    bool get_stats();
    bool sim_mode(bool on);

private:
    void run();
    bool open_once(std::string &err);
    bool handshake(std::string &err);
    void set_state(ConnState s, const std::string &detail);
    bool write_frame(const std::vector<uint8_t> &frame);
    void note_tracker_stat(const htk_tracker_stat &t);
    void note_orient_id(uint16_t id, bool sim);
    void expire_and_publish(bool force);

    std::unique_ptr<Transport> transport_;
    std::string target_;
    Options opts_;

    std::thread thread_;
    std::atomic<bool> running_ { false };
    std::atomic<bool> quit_ { false };
    std::atomic<ConnState> state_ { ConnState::Closed };

    Parser parser_;

    mutable std::mutex mu_;      /* guards device_, trackers_, last_error_ */
    mutable std::mutex write_mu_; /* serializes transport writes */
    DeviceInfo device_;
    std::vector<TrackerInfo> trackers_;
    std::string last_error_;
    Parser::Stats stats_ {};
};

} // namespace htk
