/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "htk_cobs.h"

size_t htk_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t out_len = 0;
    size_t code_pos = 0; /* index of the pending block-length byte */
    uint8_t code = 1;

    if (out_cap == 0) {
        return 0;
    }
    out[out_len++] = 0; /* placeholder for first code byte */

    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos = out_len;
            if (out_len >= out_cap) {
                return 0;
            }
            out[out_len++] = 0;
            code = 1;
        } else {
            if (out_len >= out_cap) {
                return 0;
            }
            out[out_len++] = in[i];
            code++;
            if (code == 0xFF) {
                out[code_pos] = code;
                code_pos = out_len;
                if (out_len >= out_cap) {
                    return 0;
                }
                out[out_len++] = 0;
                code = 1;
            }
        }
    }
    out[code_pos] = code;
    return out_len;
}

size_t htk_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap)
{
    size_t out_len = 0;
    size_t i = 0;

    while (i < len) {
        uint8_t code = in[i++];

        if (code == 0 || i + (size_t)(code - 1) > len) {
            return 0; /* malformed */
        }
        for (uint8_t j = 1; j < code; j++) {
            if (out_len >= out_cap) {
                return 0;
            }
            out[out_len++] = in[i++];
        }
        if (code < 0xFF && i < len) {
            if (out_len >= out_cap) {
                return 0;
            }
            out[out_len++] = 0;
        }
    }
    return out_len;
}
