/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "headtracker/parser.hpp"

#include <cstring>

extern "C" {
#include "htk_frame.h"
}

namespace htk {

void Parser::feed(const uint8_t *data, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        const uint8_t b = data[i];

        if (b == 0x00) {
            if (discarding_) {
                discarding_ = false; /* resynchronized */
            } else if (buf_len_ > 0) {
                process_block();
            }
            /* empty frame (00 00): discarded silently per spec §1.2 */
            buf_len_ = 0;
            continue;
        }
        if (discarding_) {
            continue;
        }
        if (buf_len_ >= sizeof(buf_)) {
            /* Block longer than any legal frame: garbage or a mid-stream
             * attach inside binary noise. Skip to the next delimiter. */
            discarding_ = true;
            buf_len_ = 0;
            stats_.oversize_resyncs++;
            continue;
        }
        buf_[buf_len_++] = b;
    }
}

void Parser::reset()
{
    buf_len_ = 0;
    discarding_ = false;
}

/* memcpy into the packed struct: the payload buffer has no alignment
 * guarantees and strict-aliasing casts would be UB. */
template <typename T, typename Fn>
static bool dispatch_exact(const uint8_t *payload, size_t len, const Fn &cb)
{
    if (len != sizeof(T)) {
        return false;
    }
    if (cb) {
        T pkt;
        std::memcpy(&pkt, payload, sizeof(T));
        cb(pkt);
    }
    return true;
}

void Parser::process_block()
{
    uint8_t payload[HTK_MAX_PAYLOAD];
    const size_t plen = htk_frame_decode(buf_, buf_len_, payload, sizeof(payload));

    if (plen == 0) {
        stats_.decode_errors++;
        return;
    }

    bool ok = true;
    switch (payload[0]) {
    case HTK_PKT_ORIENT:
        ok = dispatch_exact<htk_orient>(payload, plen, on_orient);
        break;
    case HTK_PKT_STATUS:
        ok = dispatch_exact<htk_status>(payload, plen, on_status);
        break;
    case HTK_PKT_TRACKER_STAT:
        ok = dispatch_exact<htk_tracker_stat>(payload, plen, on_tracker_stat);
        break;
    case HTK_PKT_RAW:
        ok = dispatch_exact<htk_raw>(payload, plen, on_raw);
        break;
    case HTK_PKT_HELLO_RESP:
        ok = dispatch_exact<htk_hello_resp>(payload, plen, on_hello_resp);
        break;
    case HTK_PKT_LOG:
        /* variable length: type byte + 0..HTK_MAX_PAYLOAD-1 text bytes */
        if (on_log) {
            on_log(std::string_view(reinterpret_cast<const char *>(payload + 1),
                                    plen - 1));
        }
        break;
    default:
        stats_.unknown_types++;
        return; /* forward compatibility: not an error, not a frame_ok */
    }

    if (ok) {
        stats_.frames_ok++;
    } else {
        stats_.size_mismatch++;
    }
}

} // namespace htk
