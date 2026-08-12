# Head Tracker Protocol Specification

**Protocol version:** 1 · **Spec status:** draft v0.1 (frozen at milestone M6) · 2026-08-12

This document specifies two wire formats:

- **Part 1 — USB host protocol**: what the dongle speaks over USB CDC-ACM to the host
  application. This is the public interface; client libraries implement exactly this.
- **Part 2 (appendix) — ESB air protocol**: what travels between the head unit and the
  dongle over the 2.4 GHz link. Internal, but specified here because the payloads are
  shared with Part 1 by design.

The C definitions in `firmware/common/protocol/htk_protocol.h` (once created) are the
single source of truth compiled into both firmware and the host library; this document and
that header must never disagree.

---

## Part 1 — USB host protocol

### 1.1 Transport

The dongle enumerates as a USB CDC-ACM virtual serial port (driverless on Windows 10+,
Linux, macOS). Baud-rate and line-coding settings are ignored — it is a USB pipe, not a
UART. Data flows as a stream of frames in both directions.

### 1.2 Framing

Each frame is **COBS-encoded** and terminated by a single `0x00` delimiter byte:

```
frame    := COBS( payload ‖ crc16_le ) ‖ 0x00
payload  := type:u8 ‖ body
crc16_le := CRC-16/CCITT-FALSE over payload, appended little-endian
```

- **COBS** (Consistent Overhead Byte Stuffing): removes all `0x00` bytes from the encoded
  region at a cost of 1 overhead byte per 254 payload bytes. A receiver resynchronizes
  after corruption or mid-stream attach by simply discarding bytes until the next `0x00`.
- **CRC-16/CCITT-FALSE**: polynomial `0x1021`, initial value `0xFFFF`, no input/output
  reflection, no final XOR. Computed over the *unencoded* payload (type byte + body),
  appended little-endian **before** COBS encoding. Check value: CRC of the ASCII bytes
  `"123456789"` is `0x29B1`.
- **Maximum payload size: 59 bytes** (⇒ encoded frame ≤ 63 bytes). Every frame therefore
  fits in a single 64-byte USB full-speed bulk packet: the dongle writes one frame per
  transfer, and since a frame always ends with `0x00`, the parser never stalls on a
  64-byte boundary and zero-length-packet handling is never needed.
- A frame whose CRC fails, whose payload is empty, or whose length does not match its
  type's defined size is discarded silently (counted in parser statistics if available).

### 1.3 Conventions

- All multi-byte integers and floats are **little-endian**. Structs are packed (no padding).
- `t_us` timestamps are the **head unit's** microsecond clock, wrapping at 2³² (~71.6 min).
  Hosts needing wall-clock alignment map them via arrival times (see PRD latency method).
- **Coordinate frame** (right-handed): body **X forward** (nose), **Y left**, **Z up**.
  World frame: Z up (gravity-aligned), X/Y defined by the most recent recenter ("front").
- **Quaternion**: Hamilton convention, order **w, x, y, z**, unit-norm, representing the
  rotation from body frame to world frame. Yaw = rotation about world Z; suggested Euler
  extraction for display: intrinsic Z-Y'-X'' (yaw-pitch-roll).
- Unknown packet types MUST be skipped without error (frame boundaries come from COBS, so
  skipping is always possible). This is the forward-compatibility mechanism.

### 1.4 Packets: dongle → host

#### `0x01 ORIENT` — orientation sample (24 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x01` |
| 1 | seq | u16 | per-source sequence number, wraps at 2¹⁶; gaps ⇒ air-side loss |
| 3 | t_us | u32 | head-unit timestamp at IMU sample time, µs |
| 7 | q_w | f32 | quaternion w |
| 11 | q_x | f32 | quaternion x |
| 15 | q_y | f32 | quaternion y |
| 19 | q_z | f32 | quaternion z |
| 23 | flags | u8 | see below |

flags: bit 0 `HW_FUSION` (0 = software VQF, 1 = LSM6DSV16X SFLP) · bit 1 `RECENTERED`
(set on every sample until the head unit has seen the host's next command, minimum one
sample; confirms RECENTER was applied) · bit 2 `BIAS_OK` (gyro bias estimate converged) ·
bit 3 `SIM` (synthetic data, not from a sensor) · bits 4–7 reserved, 0. Bits 4–5 are
earmarked as a tracker-ID field for a future multi-tracker revision.

Sent continuously at the active rate (default ~208 Hz) whenever quaternion mode is on.

#### `0x02 STATUS` — link and device health (22 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x02` |
| 1 | uptime_ms | u32 | dongle uptime |
| 5 | rx_rate | u16 | ORIENT/RAW packets received over air in the last second |
| 7 | seq_lost | u32 | cumulative packets lost (sequence gaps) since boot |
| 11 | retrans | u32 | cumulative ESB retransmissions (reported by head unit) |
| 15 | txfail | u32 | cumulative ESB transactions dropped after final retry |
| 19 | vbat_mV | u16 | head-unit battery voltage, 0 if unknown/unpowered |
| 21 | flags | u8 | bit 0 `LINK_UP` (≥1 air packet in last 500 ms) · bit 1 `SIM_ACTIVE` · rest 0 |

Sent at 1 Hz, and immediately in response to `GET_STATS`.

#### `0x03 RAW` — raw IMU sample (20 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x03` |
| 1 | seq | u16 | shares the sequence space with ORIENT |
| 3 | t_us | u32 | sample timestamp, µs |
| 7 | gyr_x/y/z | 3×s16 | raw gyro, LSB per full-scale code below |
| 13 | acc_x/y/z | 3×s16 | raw accel, LSB per full-scale code below |
| 19 | fs | u8 | low nibble: gyro FS (0=±250, 1=±500, 2=±1000, 3=±2000 dps) · high nibble: accel FS (0=±2, 1=±4, 2=±8, 3=±16 g) |

Sent when raw mode is active (`SET_MODE` 1 or 2).

#### `0x04 HELLO_RESP` — identity/version (8 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x04` |
| 1 | proto_ver | u8 | protocol version implemented by the dongle (this spec: 1) |
| 2 | fw_major | u8 | dongle firmware version |
| 3 | fw_minor | u8 | |
| 4 | fw_patch | u8 | |
| 5 | device | u8 | 1 = dongle (other values reserved) |
| 6 | caps | u16 | bit 0 quat · bit 1 raw · bit 2 sim · bit 3 hw_fusion · rest 0 |

Sent in response to `HELLO`. The dongle answers this itself (no radio round-trip), so it
works with no head unit in range.

#### `0x7F LOG` — diagnostic text (variable, ≤ 57 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x7F` |
| 1… | text | u8[] | UTF-8, not NUL-terminated (length = frame payload length − 1) |

Human-readable diagnostics; hosts may display or ignore. Never machine-parsed.

### 1.5 Packets: host → dongle

| type | name | body | action |
|---|---|---|---|
| `0x10` | HELLO | proto_ver:u8 — version the host implements | dongle replies `HELLO_RESP` |
| `0x11` | RECENTER | — | forwarded to head unit; current heading becomes yaw zero |
| `0x12` | SET_RATE | hz:u16 | forwarded; head unit clamps to nearest supported rate (52/104/208/416); actual rate observable via `STATUS.rx_rate` |
| `0x13` | SET_MODE | mode:u8 (0 = quat, 1 = raw, 2 = both) | forwarded |
| `0x14` | GET_STATS | — | dongle sends `STATUS` immediately |
| `0x1F` | SIM_MODE | on:u8 (0/1) | dongle-local: emit synthetic ORIENT (slow yaw sweep, `SIM` flag set) at the active rate, ignoring the radio |

Forwarding semantics: `RECENTER` / `SET_RATE` / `SET_MODE` are queued as ESB ACK payloads
(see Part 2) and delivered on the next air transaction (≤ ~5 ms with a live link). Delivery
of RECENTER is confirmed by the `RECENTERED` flag in subsequent ORIENT packets. Commands
queued while the link is down are held (latest of each type wins) until the link returns.

### 1.6 Session behavior

1. Host opens the port, sends `HELLO`, checks `proto_ver` in `HELLO_RESP`
   (major-version equality required; this spec is version 1).
2. Dongle streams ORIENT as soon as air packets arrive — streaming does not wait for
   HELLO; the handshake is for the host's benefit only.
3. `STATUS` arrives at 1 Hz regardless of mode.
4. All settings are volatile; hosts should re-apply desired rate/mode after connect.

### 1.7 Worked examples (byte-exact)

CRC values below are normative test vectors for implementations.

**RECENTER** — payload `11`, CRC16 = `0xE3E0`:

```
payload+crc : 11 E0 E3
frame       : 04 11 E0 E3 00
```

**HELLO** (host, proto_ver 1) — payload `10 01`, CRC16 = `0x0E5D`:

```
payload+crc : 10 01 5D 0E
frame       : 05 10 01 5D 0E 00
```

**HELLO_RESP** (proto 1, fw 0.1.0, device 1, caps quat+raw) — CRC16 = `0x4E00`:

```
payload     : 04 01 00 01 00 01 03 00
payload+crc : 04 01 00 01 00 01 03 00 00 4E
frame       : 03 04 01 02 01 03 01 03 01 02 4E 00
```

(Note how COBS turns the four zero bytes into chain codes — a good parser test.)

**ORIENT** — seq 42, t_us 1 000 000, identity quaternion (w=1), flags `BIAS_OK`;
CRC16 = `0xD9D7`:

```
payload     : 01 2A 00 40 42 0F 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00 00 00 04
frame       : 03 01 2A 04 40 42 0F 01 01 03 80 3F 01 01 01 01 01 01 01 01 01 01 01 04
              04 D7 D9 00
```

### 1.8 Versioning rules

- `proto_ver` bumps only on breaking changes (field layout/meaning of existing packets).
- Adding new packet types or defining reserved flag bits is **not** breaking — clients
  must skip unknown types and mask unknown bits.
- Frozen at milestone M6 as v1.0; until then this draft may change without a bump.

---

## Part 2 — Appendix: ESB air protocol

Internal to the firmware pair; hosts never see this layer.

### 2.1 Radio configuration (constants in `firmware/common/radio/`)

| parameter | value |
|---|---|
| protocol | Nordic ESB, dynamic payload length, max 32 B |
| data rate | 2 Mbps |
| RF channel | 77 (2477 MHz — above Wi-Fi ch. 13; single fixed channel in v1) |
| base address 0 | `A9 5E 3C D7` + prefix `48` (pipe 0) |
| ESB CRC | 16-bit (radio-level, independent of the USB CRC) |
| retransmit | count 2, delay 300 µs (start-to-start) |
| roles | head unit = PTX, dongle = PRX |

Latency-first policy: after the final failed retry the packet is **dropped** — the next
~4.8 ms sample supersedes it. The head unit counts retransmissions and final failures and
ships the totals in a periodic status payload so they surface in USB `STATUS`.

### 2.2 Air payloads

Air payloads are the **same bytes as the USB payload** (type byte + body) with **no COBS
and no CRC16** — ESB's own 16-bit CRC and fixed packet boundaries make both redundant.
The dongle's forwarding job is therefore: prepend nothing, append CRC16, COBS-encode, send.

- Head → dongle: `ORIENT` (24 B) and/or `RAW` (20 B) at the active rate; a compact status
  payload (retrans/txfail/vbat counters) every second, merged by the dongle into USB `STATUS`.
- Dongle → head (via **ACK payload**): host commands `RECENTER` / `SET_RATE` / `SET_MODE`
  forwarded verbatim. The dongle pre-queues the payload with `esb_write_payload()`; it
  rides out on the ACK of the next received packet. At 208 Hz that bounds command latency
  to ~5 ms. Only the latest pending command of each type is kept.

### 2.3 Loss accounting

- `seq` increments per transmitted sample on the head unit (shared across ORIENT/RAW).
- The dongle detects gaps modulo 2¹⁶ and accumulates `seq_lost`.
- Duplicates (ACK lost, packet retransmitted and received twice) are detected by repeated
  `seq` and dropped by the dongle — hosts never see duplicate sequence numbers.

### 2.4 Future (reserved, not in v1)

- Channel hopping on sustained loss (3-channel table; requires a resync rule).
- Multiple trackers: additional pipes/prefixes + tracker ID in ORIENT flags bits 4–5.
- Pairing/whitening of addresses.
