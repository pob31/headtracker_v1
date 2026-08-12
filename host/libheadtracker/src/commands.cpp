/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "headtracker/commands.hpp"

#include <cstring>

extern "C" {
#include "htk_frame.h"
}

namespace htk {

static std::vector<uint8_t> frame_from(const void *payload, size_t len)
{
    std::vector<uint8_t> out(HTK_MAX_FRAME);
    const size_t n = htk_frame_encode(static_cast<const uint8_t *>(payload),
                                      len, out.data(), out.size());
    out.resize(n); /* n == 0 -> empty vector signals encoding error */
    return out;
}

static bool valid_target(uint16_t id)
{
    return id != HTK_ID_INVALID && id != HTK_ID_SIM;
}

std::vector<uint8_t> encode_hello()
{
    const htk_hello p = { HTK_PKT_HELLO, HTK_PROTO_VERSION };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_get_stats()
{
    const htk_get_stats p = { HTK_PKT_GET_STATS };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_sim_mode(bool on)
{
    const htk_sim_mode p = { HTK_PKT_SIM_MODE, static_cast<uint8_t>(on ? 1 : 0) };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_reset_fusion(uint16_t tracker_id)
{
    if (!valid_target(tracker_id)) {
        return {};
    }
    const htk_cmd_id p = { HTK_PKT_RESET_FUSION, tracker_id };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_identify(uint16_t tracker_id)
{
    if (!valid_target(tracker_id)) {
        return {};
    }
    const htk_cmd_id p = { HTK_PKT_IDENTIFY, tracker_id };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_set_rate(uint16_t tracker_id, uint16_t hz)
{
    if (!valid_target(tracker_id)) {
        return {};
    }
    const htk_set_rate p = { HTK_PKT_SET_RATE, tracker_id, hz };
    return frame_from(&p, sizeof(p));
}

std::vector<uint8_t> encode_set_mode(uint16_t tracker_id, uint8_t mode)
{
    if (!valid_target(tracker_id) || mode > HTK_MODE_BOTH) {
        return {};
    }
    const htk_set_mode p = { HTK_PKT_SET_MODE, tracker_id, mode };
    return frame_from(&p, sizeof(p));
}

} // namespace htk
