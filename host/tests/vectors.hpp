/* Normative test vectors from docs/PROTOCOL.md §1.8 (draft v0.4+).
 * These byte strings are the spec; if a test here fails, either the code or
 * the document is wrong — never "fix" the vector to make a test pass.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#pragma once

#include <cstdint>
#include <string>
#include <vector>

inline std::vector<uint8_t> hexbytes(const std::string &hex)
{
    std::vector<uint8_t> out;
    std::string acc;
    for (char c : hex) {
        if (c == ' ') {
            continue;
        }
        acc.push_back(c);
        if (acc.size() == 2) {
            out.push_back(static_cast<uint8_t>(std::stoul(acc, nullptr, 16)));
            acc.clear();
        }
    }
    return out;
}

/* frames: complete, including the trailing 0x00 delimiter */
inline const std::string kFrameResetFusion = "06 11 34 12 ED 43 00";
inline const std::string kFrameSetModeRaw  = "07 13 34 12 01 EE 68 00";
inline const std::string kFrameIdentify    = "06 15 34 12 2D 9F 00";
inline const std::string kFrameHello       = "05 10 01 5D 0E 00";
inline const std::string kFrameHelloResp   = "03 04 01 02 01 03 01 17 03 B7 81 00";
inline const std::string kFrameOrient =
    "05 01 34 12 2A 04 40 42 0F 01 01 03 80 3F 01 01 01 01 01 01 01 01 01 01 "
    "01 04 02 5F F9 00";
inline const std::string kFrameTrackerStat =
    "05 05 34 12 05 01 01 02 D0 02 03 01 01 03 AC 0F 02 01 01 04 01 25 E9 00";

inline std::vector<std::vector<uint8_t>> all_valid_frames()
{
    return {
        hexbytes(kFrameResetFusion), hexbytes(kFrameSetModeRaw),
        hexbytes(kFrameIdentify),    hexbytes(kFrameHello),
        hexbytes(kFrameHelloResp),   hexbytes(kFrameOrient),
        hexbytes(kFrameTrackerStat),
    };
}
