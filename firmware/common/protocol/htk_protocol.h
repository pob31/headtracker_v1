/*
 * htk_protocol.h — head tracker wire protocol, single source of truth.
 *
 * Compiled by both firmware apps AND the host client library. Pure C99
 * (also valid C++), no Zephyr or OS dependencies. Byte layouts are
 * normative per docs/PROTOCOL.md (draft v0.4); the two must never disagree.
 *
 * All fields little-endian, structs packed. Air payloads are the same bytes
 * as USB payloads (type + body); USB adds CRC16 + COBS framing on top.
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */
#ifndef HTK_PROTOCOL_H
#define HTK_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HTK_PROTO_VERSION 1u

#define HTK_FW_MAJOR 0u
#define HTK_FW_MINOR 1u
#define HTK_FW_PATCH 0u

/* Framing (USB side) */
#define HTK_MAX_PAYLOAD   59u  /* encoded frame <= 63 B -> one USB FS bulk packet */
#define HTK_MAX_FRAME     64u  /* COBS(payload+crc) + 0x00 delimiter, worst case */
#define HTK_CRC16_CHECK   0x29B1u /* CRC-16/CCITT-FALSE of ASCII "123456789" */

/* Tracker IDs */
#define HTK_ID_INVALID 0x0000u /* never a valid target or source */
#define HTK_ID_SIM     0xFFFFu /* dongle-generated simulator stream */

/* Packet types: device -> host */
#define HTK_PKT_ORIENT       0x01u
#define HTK_PKT_STATUS       0x02u
#define HTK_PKT_RAW          0x03u
#define HTK_PKT_HELLO_RESP   0x04u
#define HTK_PKT_TRACKER_STAT 0x05u
#define HTK_PKT_LOG          0x7Fu

/* Packet types: host -> dongle (sensor-directed ones carry a specific id) */
#define HTK_PKT_HELLO        0x10u
#define HTK_PKT_RESET_FUSION 0x11u
#define HTK_PKT_SET_RATE     0x12u
#define HTK_PKT_SET_MODE     0x13u
#define HTK_PKT_GET_STATS    0x14u
#define HTK_PKT_IDENTIFY     0x15u
#define HTK_PKT_SIM_MODE     0x1Fu

/* Air-only types (never on USB) */
#define HTK_PKT_BEACON       0x20u /* dongle -> all head units, downlink address */
#define HTK_PKT_TSTATUS      0x21u /* head unit 1 Hz status, uplink address */

/* ORIENT flags */
#define HTK_ORIENT_HW_FUSION (1u << 0)
#define HTK_ORIENT_BIAS_OK   (1u << 1)
#define HTK_ORIENT_SIM       (1u << 2)

/* STATUS flags */
#define HTK_STATUS_SIM_ACTIVE (1u << 0)

/* TRACKER_STAT flags */
#define HTK_TSTAT_LINK_UP (1u << 0)

/* HELLO_RESP caps */
#define HTK_CAP_QUAT      (1u << 0)
#define HTK_CAP_RAW       (1u << 1)
#define HTK_CAP_SIM       (1u << 2)
#define HTK_CAP_HW_FUSION (1u << 3)
#define HTK_CAP_MULTI     (1u << 4)

/* Stream modes */
#define HTK_MODE_QUAT 0u
#define HTK_MODE_RAW  1u
#define HTK_MODE_BOTH 2u

/* Supported update rates (Hz); SET_RATE clamps to nearest */
#define HTK_RATE_DEFAULT 208u

/* Full-scale codes for RAW.fs: low nibble gyro, high nibble accel */
#define HTK_FS_GYRO_250DPS  0u
#define HTK_FS_GYRO_500DPS  1u
#define HTK_FS_GYRO_1000DPS 2u
#define HTK_FS_GYRO_2000DPS 3u
#define HTK_FS_ACC_2G  0u
#define HTK_FS_ACC_4G  1u
#define HTK_FS_ACC_8G  2u
#define HTK_FS_ACC_16G 3u
#define HTK_FS_MAKE(gyro, acc) ((uint8_t)(((acc) << 4) | ((gyro) & 0x0Fu)))

#if defined(_MSC_VER)
#define HTK_PACKED_BEGIN __pragma(pack(push, 1))
#define HTK_PACKED_END   __pragma(pack(pop))
#define HTK_PACKED
#else
#define HTK_PACKED_BEGIN
#define HTK_PACKED_END
#define HTK_PACKED __attribute__((packed))
#endif

HTK_PACKED_BEGIN

/* 0x01 — 26 bytes. Quaternion: Hamilton, w x y z, body->world, body X fwd/Y left/Z up. */
struct HTK_PACKED htk_orient {
    uint8_t  type;    /* HTK_PKT_ORIENT */
    uint16_t id;
    uint16_t seq;     /* per-tracker, shared with RAW, wraps */
    uint32_t t_us;    /* head-unit clock at IMU sample time */
    float    q_w, q_x, q_y, q_z;
    uint8_t  flags;   /* HTK_ORIENT_* */
};

/* 0x02 — 9 bytes, dongle-global, 1 Hz */
struct HTK_PACKED htk_status {
    uint8_t  type;    /* HTK_PKT_STATUS */
    uint32_t uptime_ms;
    uint16_t rx_rate;    /* air packets/s, all trackers */
    uint8_t  n_trackers; /* seen within last 3 s */
    uint8_t  flags;      /* HTK_STATUS_* */
};

/* 0x03 — 22 bytes */
struct HTK_PACKED htk_raw {
    uint8_t  type;    /* HTK_PKT_RAW */
    uint16_t id;
    uint16_t seq;
    uint32_t t_us;
    int16_t  gyr[3];
    int16_t  acc[3];
    uint8_t  fs;      /* HTK_FS_MAKE() */
};

/* 0x04 — 8 bytes */
struct HTK_PACKED htk_hello_resp {
    uint8_t  type;    /* HTK_PKT_HELLO_RESP */
    uint8_t  proto_ver;
    uint8_t  fw_major, fw_minor, fw_patch;
    uint8_t  device;  /* 1 = dongle */
    uint16_t caps;    /* HTK_CAP_* */
};

/* 0x05 — 20 bytes, per known tracker, 1 Hz */
struct HTK_PACKED htk_tracker_stat {
    uint8_t  type;    /* HTK_PKT_TRACKER_STAT */
    uint16_t id;
    uint32_t age_ms;
    uint16_t rate;      /* packets/s as seen by this dongle */
    uint32_t seq_lost;  /* cumulative, as seen by this dongle */
    uint16_t vbat_mV;   /* self-reported, 0 = unknown */
    uint8_t  fw_major, fw_minor, fw_patch; /* self-reported */
    uint8_t  mode;      /* HTK_MODE_*, self-reported */
    uint8_t  flags;     /* HTK_TSTAT_* */
};

/* Host -> dongle commands */
struct HTK_PACKED htk_hello       { uint8_t type; uint8_t proto_ver; };
struct HTK_PACKED htk_cmd_id      { uint8_t type; uint16_t id; };            /* RESET_FUSION, IDENTIFY */
struct HTK_PACKED htk_set_rate    { uint8_t type; uint16_t id; uint16_t hz; };
struct HTK_PACKED htk_set_mode    { uint8_t type; uint16_t id; uint8_t mode; };
struct HTK_PACKED htk_get_stats   { uint8_t type; };
struct HTK_PACKED htk_sim_mode    { uint8_t type; uint8_t on; };

/* 0x20 — air only: beacon header; an optional embedded command payload
 * (type + body, one of the host->dongle sensor commands) follows directly. */
struct HTK_PACKED htk_beacon_hdr {
    uint8_t type;       /* HTK_PKT_BEACON */
    uint8_t beacon_seq;
};

/* 0x21 — air only: head unit self-report, 1 Hz, 13 bytes */
struct HTK_PACKED htk_tstatus {
    uint8_t  type;      /* HTK_PKT_TSTATUS */
    uint16_t id;
    uint16_t vbat_mV;   /* 0 = unknown */
    uint8_t  fw_major, fw_minor, fw_patch;
    uint8_t  mode;      /* HTK_MODE_* */
    uint16_t rate;      /* configured stream rate, Hz */
    uint16_t uptime_s;
};

HTK_PACKED_END

/* Layout guards — a failure here means a compiler broke the wire format. */
#if defined(__cplusplus)
#define HTK_ASSERT_SIZE(t, n) static_assert(sizeof(t) == (n), #t " size")
#elif defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
#define HTK_ASSERT_SIZE(t, n) _Static_assert(sizeof(t) == (n), #t " size")
#else
#define HTK_ASSERT_SIZE(t, n)
#endif
HTK_ASSERT_SIZE(struct htk_orient, 26);
HTK_ASSERT_SIZE(struct htk_status, 9);
HTK_ASSERT_SIZE(struct htk_raw, 22);
HTK_ASSERT_SIZE(struct htk_hello_resp, 8);
HTK_ASSERT_SIZE(struct htk_tracker_stat, 20);
HTK_ASSERT_SIZE(struct htk_tstatus, 13);

#ifdef __cplusplus
}
#endif

#endif /* HTK_PROTOCOL_H */
