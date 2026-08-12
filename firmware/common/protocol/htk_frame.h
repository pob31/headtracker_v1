/* Frame = COBS(payload + crc16_le) + 0x00. Helpers used by the dongle TX path
 * and by any C consumer; the C++ host library reimplements the RX side as a
 * push parser but shares these definitions.
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_FRAME_H
#define HTK_FRAME_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Builds a complete frame (including trailing 0x00) from payload[0..len).
 * Returns frame length, or 0 on error (len > HTK_MAX_PAYLOAD or out_cap small).
 * out_cap of HTK_MAX_FRAME always suffices. */
size_t htk_frame_encode(const uint8_t *payload, size_t len, uint8_t *out, size_t out_cap);

/* Decodes one delimiter-free COBS block (as accumulated between 0x00 bytes).
 * On success returns payload length (>=1) with payload bytes in out;
 * returns 0 on COBS error, CRC mismatch, or undersized input. */
size_t htk_frame_decode(const uint8_t *block, size_t len, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif
