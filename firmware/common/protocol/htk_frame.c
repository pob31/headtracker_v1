/* SPDX-License-Identifier: GPL-3.0-or-later */
#include "htk_frame.h"
#include "htk_cobs.h"
#include "htk_crc16.h"
#include "htk_protocol.h"

size_t htk_frame_encode(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap)
{
    uint8_t buf[HTK_MAX_PAYLOAD + 2];

    if (len == 0 || len > HTK_MAX_PAYLOAD) {
        return 0;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = payload[i];
    }
    uint16_t crc = htk_crc16(payload, len);
    buf[len]     = (uint8_t)(crc & 0xFFu);
    buf[len + 1] = (uint8_t)(crc >> 8);

    size_t enc = htk_cobs_encode(buf, len + 2, out, out_cap);
    if (enc == 0 || enc + 1 > out_cap) {
        return 0;
    }
    out[enc] = 0x00;
    return enc + 1;
}

size_t htk_frame_decode(const uint8_t *block, size_t len, uint8_t *out, size_t out_cap)
{
    uint8_t buf[HTK_MAX_PAYLOAD + 2];

    size_t dec = htk_cobs_decode(block, len, buf, sizeof(buf));
    if (dec < 3) { /* type + crc16 minimum */
        return 0;
    }
    size_t plen = dec - 2;
    uint16_t rx_crc = (uint16_t)buf[plen] | ((uint16_t)buf[plen + 1] << 8);
    if (htk_crc16(buf, plen) != rx_crc) {
        return 0;
    }
    if (plen > out_cap) {
        return 0;
    }
    for (size_t i = 0; i < plen; i++) {
        out[i] = buf[i];
    }
    return plen;
}
