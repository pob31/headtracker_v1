/* COBS encode/decode (no 0x00 in encoded output; 0x00 is the frame delimiter).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_COBS_H
#define HTK_COBS_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Worst-case encoded size for n input bytes (excludes the 0x00 delimiter). */
#define HTK_COBS_ENC_MAX(n) ((n) + ((n) / 254u) + 1u)

/* Encodes in[0..len). Returns encoded length, or 0 if out_cap too small.
 * Does NOT append the 0x00 delimiter. */
size_t htk_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

/* Decodes a delimiter-free COBS block in[0..len). Returns decoded length,
 * or 0 on malformed input / insufficient out_cap. */
size_t htk_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t out_cap);

#ifdef __cplusplus
}
#endif

#endif
