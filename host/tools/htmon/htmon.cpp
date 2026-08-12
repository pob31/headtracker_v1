/* htmon — live monitor for a head-tracker dongle.
 *
 * Two jobs:
 *   1. the bring-up instrument: what is on the wire, which trackers are alive,
 *      how much is being lost, and does the orientation move the right way;
 *   2. the hardware-free end-to-end driver: fed an htgen capture it exercises
 *      the real parser, client and tracker table with no device present.
 *
 *   htmon --list
 *   htmon --port COM7
 *   htmon --replay dump.bin [--fast] [--loop]
 *   htgen --trackers 3 | htmon --replay -
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#include "headtracker/client.hpp"
#include "headtracker/orientation.hpp"
#include "headtracker/transport.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <conio.h>
#include <windows.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace {

constexpr float kRad2Deg = 57.29577951308232f;

struct Options {
    std::string port;
    std::string replay;
    bool list = false;
    bool fast = false;
    bool loop = false;
    bool once = false;
    bool json = false;
    bool have_select = false;
    uint16_t select = 0;

    /* One-shot commands, applied once the link is Ready. */
    int identify = -1;
    int reset_fusion = -1;
    int set_rate_id = -1, set_rate_hz = 0;
    int set_mode_id = -1, set_mode_val = 0;
    int sim = -1;
};

void usage()
{
    std::printf(
        "htmon - head tracker monitor\n"
        "\n"
        "  --list                 list serial ports and exit\n"
        "  --port PATH            open a serial port (COM7, /dev/ttyACM0);\n"
        "                         omitted, the dongle is auto-discovered\n"
        "  --replay FILE          replay an htgen capture instead ('-' = stdin)\n"
        "  --fast                 replay with no pacing\n"
        "  --loop                 restart the capture at EOF\n"
        "  --once                 run to the end / 10 s, print a summary, exit\n"
        "  --tracker ID           select a tracker for the orientation readout\n"
        "  --json                 one JSON object per second instead of a table\n"
        "  --identify ID          flash a tracker's LED\n"
        "  --reset ID             reset a tracker's fusion filter\n"
        "  --set-rate ID HZ       set a tracker's rate (52/104/208/416)\n"
        "  --set-mode ID MODE     0=quat 1=raw 2=both\n"
        "  --sim on|off           dongle-local synthetic tracker\n"
        "\n"
        "keys: r recenter  b boresight  z clear references  i identify  q quit\n");
}

uint16_t parse_id(const char *s)
{
    return (uint16_t) std::strtoul(s, nullptr, 0); /* accepts 0x1234 and 4660 */
}

/* ---- latest sample, handed from the reader thread to the display ---- */
struct Latest {
    std::mutex mu;
    bool have = false;
    uint16_t id = 0;
    htk::Quat raw;
    uint32_t seq = 0;
    uint8_t flags = 0;
    uint64_t count = 0;
};

Latest g_latest;
htk::Recenterer g_ref;       /* touched only on the reader thread */
std::mutex g_ref_mu;         /* ...except by key handling, so guard it */
std::atomic<int> g_pending_ref { 0 }; /* 1 = recenter, 2 = boresight, 3 = clear */

/* ---- portable single-key input (disabled when stdin is the data stream) ---- */
class KeyReader {
public:
    explicit KeyReader(bool enabled) : enabled_(enabled)
    {
#if !defined(_WIN32)
        if (enabled_ && ::isatty(STDIN_FILENO)) {
            if (tcgetattr(STDIN_FILENO, &saved_) == 0) {
                struct termios raw = saved_;
                raw.c_lflag &= ~(unsigned) (ICANON | ECHO);
                raw.c_cc[VMIN] = 0;
                raw.c_cc[VTIME] = 0;
                tcsetattr(STDIN_FILENO, TCSANOW, &raw);
                restore_ = true;
            }
        }
#endif
    }

    ~KeyReader()
    {
#if !defined(_WIN32)
        if (restore_) {
            tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
        }
#endif
    }

    int get()
    {
        if (!enabled_) {
            return -1;
        }
#if defined(_WIN32)
        return _kbhit() ? _getch() : -1;
#else
        unsigned char c;
        return (::read(STDIN_FILENO, &c, 1) == 1) ? (int) c : -1;
#endif
    }

private:
    bool enabled_ = false;
#if !defined(_WIN32)
    struct termios saved_ {};
    bool restore_ = false;
#endif
};

void enable_ansi()
{
#if defined(_WIN32)
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    DWORD mode = 0;
    if (GetConsoleMode(h, &mode)) {
        SetConsoleMode(h, mode | 0x0004 /* ENABLE_VIRTUAL_TERMINAL_PROCESSING */);
    }
#endif
}

const char *mode_name(uint8_t m)
{
    switch (m) {
    case HTK_MODE_QUAT: return "quat";
    case HTK_MODE_RAW:  return "raw";
    case HTK_MODE_BOTH: return "both";
    default:            return "?";
    }
}

} // namespace

int main(int argc, char **argv)
{
    Options opt;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](const char *what) -> const char * {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "htmon: %s needs a value\n", what);
                std::exit(2);
            }
            return argv[++i];
        };

        if (a == "--help" || a == "-h") { usage(); return 0; }
        else if (a == "--list")      { opt.list = true; }
        else if (a == "--fast")      { opt.fast = true; }
        else if (a == "--loop")      { opt.loop = true; }
        else if (a == "--once")      { opt.once = true; }
        else if (a == "--json")      { opt.json = true; }
        else if (a == "--port")      { opt.port = next("--port"); }
        else if (a == "--replay")    { opt.replay = next("--replay"); }
        else if (a == "--tracker")   { opt.select = parse_id(next("--tracker")); opt.have_select = true; }
        else if (a == "--identify")  { opt.identify = (int) parse_id(next("--identify")); }
        else if (a == "--reset")     { opt.reset_fusion = (int) parse_id(next("--reset")); }
        else if (a == "--sim")       { opt.sim = std::strcmp(next("--sim"), "on") == 0 ? 1 : 0; }
        else if (a == "--set-rate")  { opt.set_rate_id = (int) parse_id(next("--set-rate"));
                                       opt.set_rate_hz = (int) parse_id(next("--set-rate HZ")); }
        else if (a == "--set-mode")  { opt.set_mode_id = (int) parse_id(next("--set-mode"));
                                       opt.set_mode_val = (int) parse_id(next("--set-mode MODE")); }
        else {
            std::fprintf(stderr, "htmon: unknown argument '%s'\n", a.c_str());
            usage();
            return 2;
        }
    }

    if (opt.list) {
        const auto ports = htk::enumerate_ports();
        if (ports.empty()) {
            std::printf("no serial ports found\n");
            return 0;
        }
        for (const auto &p : ports) {
            std::printf("%-24s %s%s\n", p.path.c_str(), p.description.c_str(),
                        htk::looks_like_dongle(p) ? "   <- looks like a dongle" : "");
        }
        return 0;
    }

    htk::Client client;

    client.on_orient = [](const htk_orient &o) {
        /* Reference changes are applied here, on the one thread that owns the
         * Recenterer, rather than from the key handler. */
        if (const int req = g_pending_ref.exchange(0)) {
            std::lock_guard<std::mutex> lk(g_ref_mu);
            const htk::Quat q = htk::quat_of(o);
            if (req == 1)      { g_ref.recenter(q); }
            else if (req == 2) { g_ref.boresight(q); }
            else               { g_ref.reset(); }
        }

        std::lock_guard<std::mutex> lk(g_latest.mu);
        g_latest.have = true;
        g_latest.id = o.id;
        g_latest.raw = htk::quat_of(o);
        g_latest.seq = o.seq;
        g_latest.flags = o.flags;
        ++g_latest.count;
    };

    client.on_state = [](htk::ConnState s, const std::string &detail) {
        std::fprintf(stderr, "[link] %s%s%s\n", htk::to_string(s),
                     detail.empty() ? "" : ": ", detail.c_str());
    };

    client.on_log = [](std::string_view s) {
        std::fprintf(stderr, "[dongle] %.*s\n", (int) s.size(), s.data());
    };

    /* Transport selection: a capture replays through FileTransport, anything
     * else opens a real port (auto-discovering when none was named). */
    std::unique_ptr<htk::Transport> transport;
    std::string target;
    bool keys_ok = true;

    if (!opt.replay.empty()) {
        /* ~31 bytes per framed ORIENT at 208 Hz per tracker; pacing to that
         * makes rate and staleness behave like the real thing. */
        const double bps = opt.fast ? 0.0 : 208.0 * 31.0;
        transport = htk::make_file(bps, opt.loop);
        target = opt.replay;
        keys_ok = (opt.replay != "-"); /* stdin is the data stream */
    } else {
        transport = htk::make_serial();
        target = opt.port;
    }

    htk::Client::Options copts;
    copts.auto_reconnect = opt.replay.empty(); /* a finished capture is not a fault */
    if (!opt.replay.empty()) {
        copts.no_data_watchdog_ms = 0; /* pacing means legitimate quiet periods */
    }

    if (!client.start(std::move(transport), target, copts)) {
        std::fprintf(stderr, "htmon: could not start\n");
        return 1;
    }

    enable_ansi();
    KeyReader keys(keys_ok && !opt.once && !opt.json);

    bool commands_sent = false;
    uint64_t frames_at_last_print = 0;
    const auto started = std::chrono::steady_clock::now();
    int exit_code = 0;

    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(200));

        const auto state = client.state();
        const auto elapsed = std::chrono::steady_clock::now() - started;

        if (state == htk::ConnState::Ready && !commands_sent) {
            commands_sent = true;
            if (opt.identify >= 0)     { client.identify((uint16_t) opt.identify); }
            if (opt.reset_fusion >= 0) { client.reset_fusion((uint16_t) opt.reset_fusion); }
            if (opt.set_rate_id >= 0)  { client.set_rate((uint16_t) opt.set_rate_id,
                                                         (uint16_t) opt.set_rate_hz); }
            if (opt.set_mode_id >= 0)  { client.set_mode((uint16_t) opt.set_mode_id,
                                                         (uint8_t) opt.set_mode_val); }
            if (opt.sim >= 0)          { client.sim_mode(opt.sim != 0); }
        }

        if (state == htk::ConnState::Incompatible) {
            std::fprintf(stderr, "htmon: %s\n", client.last_error().c_str());
            exit_code = 1;
            break;
        }

        const auto trackers = client.trackers();
        const auto stats = client.parser_stats();

        /* Pick a tracker to display: the requested one, else the first seen. */
        uint16_t shown = opt.select;
        if (!opt.have_select && !trackers.empty()) {
            shown = trackers.front().id;
        }

        htk::Quat raw;
        bool have = false;
        uint64_t count = 0;
        uint8_t flags = 0;
        {
            std::lock_guard<std::mutex> lk(g_latest.mu);
            have = g_latest.have && (trackers.empty() || g_latest.id == shown || !opt.have_select);
            raw = g_latest.raw;
            count = g_latest.count;
            flags = g_latest.flags;
        }

        htk::EulerZYX e { 0, 0, 0 };
        if (have) {
            std::lock_guard<std::mutex> lk(g_ref_mu);
            e = htk::to_euler(g_ref.apply(raw));
        }

        if (opt.json) {
            std::printf("{\"state\":\"%s\",\"trackers\":%zu,\"frames_ok\":%llu,"
                        "\"decode_errors\":%llu,\"yaw\":%.2f,\"pitch\":%.2f,\"roll\":%.2f}\n",
                        htk::to_string(state), trackers.size(),
                        (unsigned long long) stats.frames_ok,
                        (unsigned long long) stats.decode_errors,
                        e.yaw * kRad2Deg, e.pitch * kRad2Deg, e.roll * kRad2Deg);
            std::fflush(stdout);
        } else if (!opt.once) {
            std::printf("\x1b[H\x1b[2J");
            const auto d = client.device();
            std::printf("htmon  link=%-12s port=%-14s dongle fw %u.%u.%u  proto %u\n",
                        htk::to_string(state), d.port.c_str(),
                        d.fw_major, d.fw_minor, d.fw_patch, d.proto_ver);
            std::printf("frames %llu   decode-err %llu   size-mismatch %llu   unknown %llu   resync %llu\n\n",
                        (unsigned long long) stats.frames_ok,
                        (unsigned long long) stats.decode_errors,
                        (unsigned long long) stats.size_mismatch,
                        (unsigned long long) stats.unknown_types,
                        (unsigned long long) stats.oversize_resyncs);

            std::printf("   id    rate   lost   vbat    fw      mode  link  age\n");
            std::printf("  ----------------------------------------------------------\n");
            for (const auto &t : trackers) {
                std::printf("  %04X  %5u  %6u  %4u mV  %u.%u.%u  %-5s %-4s  %u ms%s\n",
                            t.id, t.rate, t.seq_lost, t.vbat_mV,
                            t.fw_major, t.fw_minor, t.fw_patch, mode_name(t.mode),
                            t.link_up ? "up" : "down", t.age_ms,
                            t.id == shown ? "  <" : "");
            }
            if (trackers.empty()) {
                std::printf("  (no trackers heard yet)\n");
            }

            std::printf("\n  tracker %04X   yaw %+7.1f   pitch %+7.1f   roll %+7.1f   %s%s\n",
                        shown, e.yaw * kRad2Deg, e.pitch * kRad2Deg, e.roll * kRad2Deg,
                        (flags & HTK_ORIENT_BIAS_OK) ? "bias-ok " : "bias-converging ",
                        (flags & HTK_ORIENT_SIM) ? "SIM" : "");
            std::printf("\n  r recenter   b boresight   z clear   i identify   q quit\n");
            std::fflush(stdout);
        }

        switch (keys.get()) {
        case 'r': g_pending_ref.store(1); break;
        case 'b': g_pending_ref.store(2); break;
        case 'z': g_pending_ref.store(3); break;
        case 'i': client.identify(shown); break;
        case 'q': case 3: goto done;
        default: break;
        }

        if (opt.once) {
            /* CI mode: stop when the capture is exhausted, or after a bounded
             * wall time so a stuck stream fails instead of hanging the job. */
            const bool ended = (state == htk::ConnState::Lost || state == htk::ConnState::Closed);
            if (ended || elapsed > std::chrono::seconds(10)) {
                std::printf("htmon: state=%s trackers=%zu frames_ok=%llu decode_errors=%llu\n",
                            htk::to_string(state), trackers.size(),
                            (unsigned long long) stats.frames_ok,
                            (unsigned long long) stats.decode_errors);
                if (stats.frames_ok == 0) {
                    std::fprintf(stderr, "htmon: no valid frames decoded\n");
                    exit_code = 1;
                }
                if (trackers.empty()) {
                    std::fprintf(stderr, "htmon: no trackers seen\n");
                    exit_code = 1;
                }
                break;
            }
        }
        frames_at_last_print = count;
        (void) frames_at_last_print;
    }

done:
    client.stop();
    return exit_code;
}
