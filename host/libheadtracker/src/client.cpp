/* htk::Client — dongle session and reader thread.
 *
 * Contains NO platform code by design: everything OS-specific lives behind
 * Transport, so this file is identical on every target and can be exercised
 * end to end against MemTransport with no hardware.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "headtracker/client.hpp"

#include "headtracker/commands.hpp"

#include <algorithm>
#include <chrono>

namespace htk {
namespace {

uint64_t now_ms()
{
    using namespace std::chrono;
    return (uint64_t) duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
}

} // namespace

const char *to_string(ConnState s)
{
    switch (s) {
    case ConnState::Closed:       return "closed";
    case ConnState::Opening:      return "opening";
    case ConnState::Handshaking:  return "handshaking";
    case ConnState::Ready:        return "ready";
    case ConnState::Incompatible: return "incompatible";
    case ConnState::Lost:         return "lost";
    }
    return "?";
}

Client::~Client()
{
    stop();
}

bool Client::start(std::unique_ptr<Transport> transport, std::string target, Options opts)
{
    if (running_.load(std::memory_order_acquire)) {
        return false;
    }
    if (thread_.joinable()) {
        thread_.join(); /* a previous run that already finished */
    }

    transport_ = std::move(transport);
    target_ = std::move(target);
    opts_ = opts;

    quit_.store(false, std::memory_order_release);
    running_.store(true, std::memory_order_release);
    thread_ = std::thread([this] { run(); });
    return true;
}

bool Client::start(std::string target, Options opts)
{
    return start(make_serial(), std::move(target), opts);
}

void Client::stop()
{
    quit_.store(true, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(false, std::memory_order_release);
    if (transport_) {
        transport_->close();
    }
    set_state(ConnState::Closed, {});
}

std::string Client::last_error() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return last_error_;
}

DeviceInfo Client::device() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return device_;
}

std::vector<TrackerInfo> Client::trackers() const
{
    /* Same corroboration gate as on_trackers: the pull and push views of the
     * tracker list must never disagree, or a UI that polls would offer the
     * user an id that the callback path considers noise. */
    std::lock_guard<std::mutex> lk(mu_);
    std::vector<TrackerInfo> visible;
    visible.reserve(trackers_.size());
    for (const auto &t : trackers_) {
        if (t.orient_count >= opts_.promote_after_orients) {
            visible.push_back(t);
        }
    }
    return visible;
}

Parser::Stats Client::parser_stats() const
{
    std::lock_guard<std::mutex> lk(mu_);
    return stats_;
}

void Client::set_state(ConnState s, const std::string &detail)
{
    const ConnState prev = state_.exchange(s, std::memory_order_acq_rel);
    if (!detail.empty()) {
        std::lock_guard<std::mutex> lk(mu_);
        last_error_ = detail;
    }
    if (prev != s && on_state) {
        on_state(s, detail);
    }
}

bool Client::write_frame(const std::vector<uint8_t> &frame)
{
    if (frame.empty()) {
        return false; /* encoder rejected the arguments */
    }
    std::lock_guard<std::mutex> lk(write_mu_);
    if (!transport_ || !transport_->is_open()) {
        return false;
    }
    return transport_->write(frame.data(), frame.size());
}

bool Client::identify(uint16_t id)                { return write_frame(encode_identify(id)); }
bool Client::reset_fusion(uint16_t id)            { return write_frame(encode_reset_fusion(id)); }
bool Client::set_rate(uint16_t id, uint16_t hz)   { return write_frame(encode_set_rate(id, hz)); }
bool Client::set_mode(uint16_t id, uint8_t mode)  { return write_frame(encode_set_mode(id, mode)); }
bool Client::get_stats()                          { return write_frame(encode_get_stats()); }
bool Client::sim_mode(bool on)                    { return write_frame(encode_sim_mode(on)); }

void Client::note_tracker_stat(const htk_tracker_stat &t)
{
    std::lock_guard<std::mutex> lk(mu_);
    auto it = std::find_if(trackers_.begin(), trackers_.end(),
                           [&](const TrackerInfo &x) { return x.id == t.id; });
    if (it == trackers_.end()) {
        TrackerInfo fresh;
        fresh.id = t.id;
        trackers_.push_back(fresh);
        it = trackers_.end() - 1;
    }
    it->age_ms = t.age_ms;
    it->rate = t.rate;
    it->seq_lost = t.seq_lost;
    it->vbat_mV = t.vbat_mV;
    it->fw_major = t.fw_major;
    it->fw_minor = t.fw_minor;
    it->fw_patch = t.fw_patch;
    it->mode = t.mode;
    it->link_up = (t.flags & kKnownTstatFlags & HTK_TSTAT_LINK_UP) != 0;
    it->last_seen_ms = now_ms();
    /* A TRACKER_STAT is corroboration in itself: promote immediately. */
    if (it->orient_count < 0xFFFFFFFFu) {
        it->orient_count = 0xFFFFFFFFu;
    }
}

void Client::note_orient_id(uint16_t id, bool sim)
{
    std::lock_guard<std::mutex> lk(mu_);
    auto it = std::find_if(trackers_.begin(), trackers_.end(),
                           [&](const TrackerInfo &x) { return x.id == id; });
    if (it == trackers_.end()) {
        /* §1.4: any id seen in ORIENT is available, not just those that have
         * had a TRACKER_STAT yet -- a tracker streams for up to a second
         * before its first status arrives. */
        TrackerInfo fresh;
        fresh.id = id;
        fresh.sim = sim;
        trackers_.push_back(fresh);
        it = trackers_.end() - 1;
    }
    it->sim = sim;
    it->last_seen_ms = now_ms();
    if (it->orient_count < 0xFFFFFFFFu) {
        ++it->orient_count;
    }
}

void Client::expire_and_publish(bool force)
{
    std::vector<TrackerInfo> visible;
    bool changed = false;

    {
        std::lock_guard<std::mutex> lk(mu_);
        const uint64_t now = now_ms();

        const size_t before = trackers_.size();
        trackers_.erase(std::remove_if(trackers_.begin(), trackers_.end(),
                                       [&](const TrackerInfo &t) {
                                           return now - t.last_seen_ms >
                                                  (uint64_t) opts_.tracker_expiry_ms;
                                       }),
                        trackers_.end());
        changed = trackers_.size() != before;

        for (const auto &t : trackers_) {
            if (t.orient_count >= opts_.promote_after_orients) {
                visible.push_back(t);
            }
        }
    }

    if ((changed || force) && on_trackers) {
        on_trackers(visible);
    }
}

bool Client::open_once(std::string &err)
{
    std::string path = target_;
    if (path.empty()) {
        path = find_dongle();
        if (path.empty()) {
            err = "no head-tracker dongle found on any serial port";
            return false;
        }
    }

    if (!transport_->open(path, err)) {
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(mu_);
        device_ = DeviceInfo{};
        device_.port = path;
    }
    parser_.reset();
    return true;
}

bool Client::handshake(std::string &err)
{
    bool answered = false;
    bool ok = false;
    htk_hello_resp resp{};

    parser_.on_hello_resp = [&](const htk_hello_resp &h) {
        answered = true;
        resp = h;
        ok = compatible(h);
    };

    const auto hello = encode_hello();
    const uint64_t deadline = now_ms() + 500;
    uint64_t next_hello = 0;
    int sent = 0;

    uint8_t buf[512];
    while (now_ms() < deadline && !answered && !quit_.load(std::memory_order_acquire)) {
        if (now_ms() >= next_hello && sent < 3) {
            if (!write_frame(hello)) {
                err = "could not send HELLO";
                return false;
            }
            next_hello = now_ms() + 100;
            ++sent;
        }

        const int n = transport_->read(buf, sizeof(buf), opts_.read_timeout_ms);
        if (n < 0) {
            err = "port closed during handshake";
            return false;
        }
        if (n > 0) {
            parser_.feed(buf, (size_t) n);
        }
    }

    parser_.on_hello_resp = nullptr;

    if (!answered) {
        /* §1.7: the dongle answers HELLO itself, with no radio round trip, so
         * silence here means the port is not a dongle -- not that no tracker
         * is in range. */
        err = "no HELLO_RESP (is this the right port?)";
        return false;
    }

    {
        std::lock_guard<std::mutex> lk(mu_);
        device_.proto_ver = resp.proto_ver;
        device_.fw_major = resp.fw_major;
        device_.fw_minor = resp.fw_minor;
        device_.fw_patch = resp.fw_patch;
        device_.device = resp.device;
        device_.caps = (uint16_t) (resp.caps & (HTK_CAP_QUAT | HTK_CAP_RAW | HTK_CAP_SIM |
                                                HTK_CAP_HW_FUSION | HTK_CAP_MULTI));
    }

    if (!ok) {
        /* §1.9 allows refuse-or-warn; refusing is right here, because a major
         * bump may change the ORIENT byte layout and silently feeding a
         * mis-parsed quaternion into a binaural renderer is worse than no
         * tracking at all. */
        err = "dongle speaks protocol v" + std::to_string((int) resp.proto_ver) +
              ", this build implements v" + std::to_string((int) kProtocolVersion);
        return false;
    }
    return true;
}

void Client::run()
{
    int backoff = opts_.reconnect_ms;

    parser_.on_orient = [this](const htk_orient &o) {
        note_orient_id(o.id, (o.flags & kKnownOrientFlags & HTK_ORIENT_SIM) != 0);
        if (on_orient) {
            on_orient(o);
        }
    };
    parser_.on_tracker_stat = [this](const htk_tracker_stat &t) { note_tracker_stat(t); };
    parser_.on_log = [this](std::string_view s) {
        if (on_log) {
            on_log(s);
        }
    };

    while (!quit_.load(std::memory_order_acquire)) {
        set_state(ConnState::Opening, {});

        std::string err;
        if (!open_once(err)) {
            set_state(ConnState::Lost, err);
            if (!opts_.auto_reconnect) {
                break;
            }
            /* Sleep in slices so stop() stays responsive. */
            for (int slept = 0; slept < backoff && !quit_.load(std::memory_order_acquire); slept += 50) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            backoff = (std::min)(backoff * 2, opts_.reconnect_max_ms);
            continue;
        }

        set_state(ConnState::Handshaking, {});
        if (!handshake(err)) {
            const bool incompatible = err.find("protocol v") != std::string::npos;
            transport_->close();
            set_state(incompatible ? ConnState::Incompatible : ConnState::Lost, err);
            if (incompatible || !opts_.auto_reconnect) {
                /* Retrying a version mismatch would just spin: it needs a
                 * firmware or application update, not another attempt. */
                break;
            }
            for (int slept = 0; slept < backoff && !quit_.load(std::memory_order_acquire); slept += 50) {
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            backoff = (std::min)(backoff * 2, opts_.reconnect_max_ms);
            continue;
        }

        backoff = opts_.reconnect_ms;
        set_state(ConnState::Ready, {});

        /* Populate the tracker list now rather than waiting up to a second
         * for the next periodic TRACKER_STAT. */
        get_stats();

        uint64_t last_data = now_ms();
        uint64_t last_sweep = now_ms();
        uint8_t buf[512];

        while (!quit_.load(std::memory_order_acquire)) {
            const int n = transport_->read(buf, sizeof(buf), opts_.read_timeout_ms);
            if (n < 0) {
                err = "port closed";
                break;
            }
            if (n > 0) {
                parser_.feed(buf, (size_t) n);
                last_data = now_ms();
                /* Publish counters per read chunk, not on the 1 Hz sweep: a
                 * short or fast stream can finish between sweeps, and a
                 * monitor reporting stale zeros for a stream it just consumed
                 * is worse than useless. One uncontended lock per chunk (not
                 * per frame) is nothing next to the read itself. */
                std::lock_guard<std::mutex> lk(mu_);
                stats_ = parser_.stats();
            } else if (opts_.no_data_watchdog_ms > 0 &&
                       now_ms() - last_data > (uint64_t) opts_.no_data_watchdog_ms) {
                /* An unplug does not reliably surface as a read error on every
                 * OS; silence from a device that streams at 1 Hz minimum
                 * (STATUS) is unambiguous. */
                err = "no data from the dongle";
                break;
            }

            if (now_ms() - last_sweep >= 1000) {
                last_sweep = now_ms();
                expire_and_publish(true);
            }
        }

        transport_->close();
        if (!quit_.load(std::memory_order_acquire)) {
            set_state(ConnState::Lost, err);
            if (!opts_.auto_reconnect) {
                break;
            }
        }
    }

    parser_.on_orient = nullptr;
    parser_.on_tracker_stat = nullptr;
    parser_.on_log = nullptr;

    running_.store(false, std::memory_order_release);
}

} // namespace htk
