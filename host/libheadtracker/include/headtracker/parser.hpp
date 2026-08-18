/* htk::Parser — push parser for the dongle's USB stream (PROTOCOL.md Part 1).
 *
 * Feed it raw serial bytes in any chunking; it emits typed packet callbacks.
 * Guarantees for consumer apps:
 *   - never throws on wire data, never allocates per-byte, bounded memory
 *   - malformed input (bad COBS, bad CRC, wrong size, oversize) is counted in
 *     Stats and silently resynchronized — garbage cannot crash the app
 *   - version tolerance: unknown packet types are counted and skipped, so a
 *     newer dongle never breaks an older client (see PROTOCOL.md §1.8)
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <string_view>

#include "htk_protocol.h"

namespace htk {

/* Protocol version this library implements. Check against HELLO_RESP. */
inline constexpr uint8_t kProtocolVersion = HTK_PROTO_VERSION;

/* Major-version equality rule from PROTOCOL.md §1.8. */
inline constexpr bool compatible(const htk_hello_resp &h)
{
    return h.proto_ver == kProtocolVersion;
}

/* Flag bits this library version understands; mask before interpreting so a
 * future protocol revision's new bits are ignored, not misread. */
inline constexpr uint8_t kKnownOrientFlags =
    HTK_ORIENT_HW_FUSION | HTK_ORIENT_BIAS_OK | HTK_ORIENT_SIM |
    HTK_ORIENT_REST | HTK_ORIENT_TAP;
inline constexpr uint8_t kKnownStatusFlags = HTK_STATUS_SIM_ACTIVE;
inline constexpr uint8_t kKnownTstatFlags  = HTK_TSTAT_LINK_UP;

class Parser {
public:
    struct Stats {
        uint64_t frames_ok = 0;        /* accepted + dispatched */
        uint64_t decode_errors = 0;    /* COBS malformed or CRC mismatch */
        uint64_t size_mismatch = 0;    /* known type, wrong payload length */
        uint64_t unknown_types = 0;    /* skipped for forward compatibility */
        uint64_t oversize_resyncs = 0; /* block exceeded max frame; discarded */
    };

    /* Typed packet callbacks; leave unset to ignore a type. Structs are the
     * exact wire structs from htk_protocol.h (little-endian packed; this
     * library only targets little-endian hosts, as the firmware asserts). */
    std::function<void(const htk_orient &)> on_orient;
    std::function<void(const htk_status &)> on_status;
    std::function<void(const htk_tracker_stat &)> on_tracker_stat;
    std::function<void(const htk_raw &)> on_raw;
    std::function<void(const htk_hello_resp &)> on_hello_resp;
    std::function<void(std::string_view)> on_log; /* UTF-8, not NUL-terminated */

    /* Consume a chunk of bytes from the serial port (any size, any split). */
    void feed(const uint8_t *data, size_t len);

    const Stats &stats() const { return stats_; }

    /* Drop any partial frame state (e.g. after reopening the port). */
    void reset();

private:
    void process_block();

    /* Longest legal encoded block is HTK_MAX_FRAME-1 bytes (delimiter not
     * buffered). Anything longer is garbage: discard until next delimiter. */
    uint8_t buf_[HTK_MAX_FRAME];
    size_t buf_len_ = 0;
    bool discarding_ = false;
    Stats stats_;
};

} // namespace htk
