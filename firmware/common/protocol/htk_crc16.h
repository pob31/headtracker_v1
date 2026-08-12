/* CRC-16/CCITT-FALSE: poly 0x1021, init 0xFFFF, no reflection, no xorout.
 * Check value: crc of ASCII "123456789" == 0x29B1 (HTK_CRC16_CHECK).
 * SPDX-License-Identifier: GPL-3.0-or-later */
#ifndef HTK_CRC16_H
#define HTK_CRC16_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint16_t htk_crc16(const uint8_t *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif
