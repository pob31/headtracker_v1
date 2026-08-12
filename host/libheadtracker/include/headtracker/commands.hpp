/* Host -> dongle command encoding (PROTOCOL.md §1.5), ready-to-write frames.
 *
 * Sensor-directed commands must target one concrete tracker: id 0x0000 and
 * 0xFFFF are rejected here (returns an empty vector) exactly as the dongle
 * would silently drop them — catching the mistake at the API instead.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#pragma once

#include <cstdint>
#include <vector>

#include "htk_protocol.h"

namespace htk {

/* Each returns a complete frame (COBS + CRC + 0x00 delimiter) to write to the
 * serial port verbatim, or an empty vector if arguments violate the spec. */
std::vector<uint8_t> encode_hello();
std::vector<uint8_t> encode_get_stats();
std::vector<uint8_t> encode_sim_mode(bool on);
std::vector<uint8_t> encode_reset_fusion(uint16_t tracker_id);
std::vector<uint8_t> encode_identify(uint16_t tracker_id);
std::vector<uint8_t> encode_set_rate(uint16_t tracker_id, uint16_t hz);
std::vector<uint8_t> encode_set_mode(uint16_t tracker_id, uint8_t mode);

} // namespace htk
