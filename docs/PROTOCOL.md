# Head Tracker Protocol Specification

**Protocol version:** 1 · **Spec status:** draft v0.2 (frozen at milestone M6) · 2026-08-12

This document specifies two wire formats:

- **Part 1 — USB host protocol**: what the dongle speaks over USB CDC-ACM to the host
  application. This is the public interface; client libraries implement exactly this.
- **Part 2 (appendix) — ESB air protocol**: what travels between head units and the
  dongle over the 2.4 GHz link. Internal, but specified here because the payloads are
  shared with Part 1 by design.

The system is **multi-tracker and pairing-free**: the dongle receives every head unit in
range and multiplexes all of them onto the USB stream, each packet tagged with the
tracker's stable hardware ID. The audio application lists the available trackers and the
user selects one there — no pairing step when a battery dies or headphones are swapped.
A LAN bridge daemon that republishes all trackers to multiple render machines is on the
roadmap (see §2.5); it introduces no changes to this protocol.

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
- **Tracker ID** (`id:u16`): derived on each head unit from the nRF52840's factory device
  ID (FICR `DEVICEADDR[0]` lower 16 bits), so it is stable across power cycles and needs
  no provisioning. Values `0x0000` and `0xFFFF` are reserved (broadcast address and
  simulator source respectively); a head unit whose hardware value collides with a
  reserved value maps itself to `0x0001`/`0xFFFE`.
- `t_us` timestamps are the **originating head unit's** microsecond clock, wrapping at
  2³² (~71.6 min). Clocks of different trackers are unrelated. Hosts needing wall-clock
  alignment map them via arrival times (see PRD latency method).
- **Coordinate frame** (right-handed): body **X forward** (nose), **Y left**, **Z up**.
  World frame: Z up (gravity-aligned), X/Y defined by the tracker's most recent recenter
  ("front").
- **Quaternion**: Hamilton convention, order **w, x, y, z**, unit-norm, representing the
  rotation from body frame to world frame. Yaw = rotation about world Z; suggested Euler
  extraction for display: intrinsic Z-Y'-X'' (yaw-pitch-roll).
- Unknown packet types MUST be skipped without error (frame boundaries come from COBS, so
  skipping is always possible). This is the forward-compatibility mechanism.

### 1.4 Packets: dongle → host

#### `0x01 ORIENT` — orientation sample (26 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x01` |
| 1 | id | u16 | tracker ID (`0xFFFF` = simulator) |
| 3 | seq | u16 | per-tracker sequence number, wraps at 2¹⁶; gaps ⇒ air-side loss |
| 5 | t_us | u32 | head-unit timestamp at IMU sample time, µs |
| 9 | q_w | f32 | quaternion w |
| 13 | q_x | f32 | quaternion x |
| 17 | q_y | f32 | quaternion y |
| 21 | q_z | f32 | quaternion z |
| 25 | flags | u8 | see below |

flags: bit 0 `HW_FUSION` (0 = software VQF, 1 = LSM6DSV16X SFLP) · bit 1 `RECENTERED`
(set for ~250 ms of samples after a recenter is applied; confirms delivery) · bit 2
`BIAS_OK` (gyro bias estimate converged) · bit 3 `SIM` (synthetic data, not from a
sensor) · bits 4–7 reserved, 0.

Sent continuously at each tracker's active rate (default ~208 Hz) whenever quaternion
mode is on. Streams from multiple trackers interleave on the USB pipe; consumers filter
by `id`.

#### `0x02 STATUS` — dongle health (9 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x02` |
| 1 | uptime_ms | u32 | dongle uptime |
| 5 | rx_rate | u16 | total air packets received in the last second (all trackers) |
| 7 | n_trackers | u8 | trackers currently live (seen within the last 3 s) |
| 8 | flags | u8 | bit 0 `SIM_ACTIVE` · rest 0 |

Sent at 1 Hz, and immediately in response to `GET_STATS`.

#### `0x05 TRACKER_STAT` — per-tracker health (24 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x05` |
| 1 | id | u16 | tracker ID |
| 3 | age_ms | u32 | time since last packet from this tracker |
| 7 | rate | u16 | packets/s from this tracker over the last second |
| 9 | seq_lost | u32 | cumulative lost packets (sequence gaps) since dongle boot |
| 13 | retrans | u32 | cumulative ESB retransmissions (self-reported by the head unit) |
| 17 | txfail | u32 | cumulative ESB transactions dropped after final retry |
| 21 | vbat_mV | u16 | head-unit battery voltage, 0 if unknown/unpowered |
| 23 | flags | u8 | bit 0 `LINK_UP` (packet within last 500 ms) · rest 0 |

Sent at 1 Hz **per known tracker**, and after `GET_STATS`. This is the packet an
application uses to build its tracker list: any `id` seen here (or in ORIENT) is
available for selection. Trackers silent for 30 s are dropped from the dongle's table
and stop appearing.

#### `0x03 RAW` — raw IMU sample (22 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x03` |
| 1 | id | u16 | tracker ID |
| 3 | seq | u16 | shares the per-tracker sequence space with ORIENT |
| 5 | t_us | u32 | sample timestamp, µs |
| 9 | gyr_x/y/z | 3×s16 | raw gyro, LSB per full-scale code below |
| 15 | acc_x/y/z | 3×s16 | raw accel, LSB per full-scale code below |
| 21 | fs | u8 | low nibble: gyro FS (0=±250, 1=±500, 2=±1000, 3=±2000 dps) · high nibble: accel FS (0=±2, 1=±4, 2=±8, 3=±16 g) |

Sent by trackers whose mode includes raw (`SET_MODE` 1 or 2).

#### `0x04 HELLO_RESP` — identity/version (8 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x04` |
| 1 | proto_ver | u8 | protocol version implemented by the dongle (this spec: 1) |
| 2 | fw_major | u8 | dongle firmware version |
| 3 | fw_minor | u8 | |
| 4 | fw_patch | u8 | |
| 5 | device | u8 | 1 = dongle (other values reserved) |
| 6 | caps | u16 | bit 0 quat · bit 1 raw · bit 2 sim · bit 3 hw_fusion · bit 4 multi-tracker · rest 0 |

Sent in response to `HELLO`. The dongle answers this itself (no radio round-trip), so it
works with no head unit in range.

#### `0x7F LOG` — diagnostic text (variable, ≤ 57 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x7F` |
| 1… | text | u8[] | UTF-8, not NUL-terminated (length = frame payload length − 1) |

Human-readable diagnostics; hosts may display or ignore. Never machine-parsed.

### 1.5 Packets: host → dongle

All tracker-directed commands carry a target `id`; `id = 0x0000` addresses **all**
trackers. Commands are idempotent by design (see delivery semantics below).

| type | name | body | action |
|---|---|---|---|
| `0x10` | HELLO | proto_ver:u8 — version the host implements | dongle replies `HELLO_RESP` |
| `0x11` | RECENTER | id:u16 | target tracker's current heading becomes its yaw zero |
| `0x12` | SET_RATE | id:u16, hz:u16 | target clamps to nearest supported rate (52/104/208/416); actual rate observable via `TRACKER_STAT.rate` |
| `0x13` | SET_MODE | id:u16, mode:u8 (0 = quat, 1 = raw, 2 = both) | forwarded to target |
| `0x14` | GET_STATS | — | dongle sends `STATUS` + one `TRACKER_STAT` per tracker immediately |
| `0x1F` | SIM_MODE | on:u8 (0/1) | dongle-local: emit synthetic ORIENT as tracker `0xFFFF` (slow yaw sweep, `SIM` flag set), alongside any real trackers |

**Delivery semantics (important):** commands travel to head units as ESB ACK payloads on
a radio address shared by all trackers, and an ACK payload is consumed by *whichever*
head unit transmits next — not necessarily the target. The dongle therefore **repeats**
each tracker-directed command in consecutive ACK slots for 100 ms (≈ 20 deliveries at
208 Hz); every head unit sees it, and units whose ID doesn't match (and isn't `0x0000`)
ignore it. Because commands are idempotent, duplicate delivery to the target is harmless.
RECENTER delivery is visible via the `RECENTERED` flag in the target's ORIENT stream;
SET_RATE/SET_MODE take effect within the repetition window with a live link. Commands
issued while the target's link is down are effectively lost — re-issue after
`TRACKER_STAT` shows `LINK_UP` again.

### 1.6 Session behavior

1. Host opens the port, sends `HELLO`, checks `proto_ver` in `HELLO_RESP`
   (major-version equality required; this spec is version 1).
2. Dongle streams ORIENT from every tracker in range as soon as air packets arrive —
   streaming neither waits for HELLO nor for any selection; the handshake is for the
   host's benefit only.
3. The application builds its tracker list from `TRACKER_STAT` (1 Hz per tracker) and
   lets the user pick; "selection" is purely client-side filtering by `id`.
4. `STATUS` arrives at 1 Hz regardless of mode.
5. All settings are volatile; hosts should re-apply desired rate/mode after connect.

### 1.7 Worked examples (byte-exact)

CRC values below are normative test vectors for implementations.

**RECENTER, broadcast** (`id 0x0000`) — payload `11 00 00`, CRC16 = `0xB8CF`:

```
payload+crc : 11 00 00 CF B8
frame       : 02 11 01 03 CF B8 00
```

**RECENTER, tracker `0x1234`** — payload `11 34 12`, CRC16 = `0x43ED`:

```
payload+crc : 11 34 12 ED 43
frame       : 06 11 34 12 ED 43 00
```

**HELLO** (host, proto_ver 1) — payload `10 01`, CRC16 = `0x0E5D`:

```
payload+crc : 10 01 5D 0E
frame       : 05 10 01 5D 0E 00
```

**HELLO_RESP** (proto 1, fw 0.1.0, device 1, caps quat+raw+sim+multi = `0x0017`) —
CRC16 = `0x81B7`:

```
payload     : 04 01 00 01 00 01 17 00
frame       : 03 04 01 02 01 03 01 17 03 B7 81 00
```

(Note how COBS turns the zero bytes into chain codes — a good parser test.)

**ORIENT** — tracker `0x1234`, seq 42, t_us 1 000 000, identity quaternion (w=1),
flags `BIAS_OK`; CRC16 = `0x9999`:

```
payload     : 01 34 12 2A 00 40 42 0F 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00
              00 00 04
frame       : 05 01 34 12 2A 04 40 42 0F 01 01 03 80 3F 01 01 01 01 01 01 01 01 01
              01 01 04 04 99 99 00
```

**TRACKER_STAT** — tracker `0x1234`, age 5 ms, 208 pkt/s, 3 lost, 17 retransmits,
3 tx-fail, vbat 4012 mV, `LINK_UP`; CRC16 = `0xEC95`:

```
payload     : 05 34 12 05 00 00 00 D0 00 03 00 00 00 11 00 00 00 03 00 00 00 AC 0F 01
frame       : 05 05 34 12 05 01 01 02 D0 02 03 01 01 02 11 01 01 02 03 01 01 06 AC 0F
              01 95 EC 00
```

### 1.8 Versioning rules

- `proto_ver` bumps only on breaking changes (field layout/meaning of existing packets).
- Adding new packet types or defining reserved flag bits is **not** breaking — clients
  must skip unknown types and mask unknown bits.
- Frozen at milestone M6 as v1.0; until then this draft may change without a bump.

---

## Part 2 — Appendix: ESB air protocol

Internal to the firmware; hosts never see this layer.

### 2.1 Radio configuration (constants in `firmware/common/radio/`)

| parameter | value |
|---|---|
| protocol | Nordic ESB, dynamic payload length, max 32 B |
| data rate | 2 Mbps |
| RF channel | 77 (2477 MHz — above Wi-Fi ch. 13; single fixed channel in v1) |
| base address 0 | `A9 5E 3C D7` + prefix `48` (pipe 0, **shared by all head units**) |
| ESB CRC | 16-bit (radio-level, independent of the USB CRC) |
| retransmit | count 2, delay 300 µs (start-to-start) |
| roles | head units = PTX (any number, no pairing), dongle = PRX |

**Shared-address multi-PTX:** all head units transmit on the same address; the dongle
distinguishes them by the `id` field inside every payload. Transmissions from different
trackers can collide on air; ESB's ACK-or-retransmit handles it. At 208 Hz a full
transaction is ~250–400 µs, so each tracker occupies well under 10% air time and 2–4
concurrent trackers see only occasional first-attempt collisions, absorbed by the retry
budget and — at worst — counted as a dropped sample. Beyond ~4 trackers, reduce
per-tracker rate via `SET_RATE` (broadcast `id 0x0000`).

Latency-first policy: after the final failed retry the packet is **dropped** — the next
~4.8 ms sample supersedes it. Each head unit counts retransmissions and final failures
and ships the totals in its periodic status payload so they surface in `TRACKER_STAT`.

### 2.2 Air payloads

Air payloads are the **same bytes as the USB payload** (type byte + body) with **no COBS
and no CRC16** — ESB's own 16-bit CRC and fixed packet boundaries make both redundant.
The dongle's forwarding job is therefore: prepend nothing, append CRC16, COBS-encode,
send. Every air payload already carries the tracker `id`.

- Head → dongle: `ORIENT` (26 B) and/or `RAW` (22 B) at the active rate; a compact status
  payload (retrans/txfail/vbat counters) every second, merged by the dongle into that
  tracker's `TRACKER_STAT`.
- Dongle → heads (via **ACK payload**): host commands `RECENTER` / `SET_RATE` /
  `SET_MODE` forwarded verbatim, repeated across ACK slots for 100 ms as described in
  §1.5 — an ACK payload is consumed by whichever PTX ACKs next, so repetition plus
  ID-filtering on the head units substitutes for routing. Only the latest pending
  command of each type is kept in the repeat queue.

### 2.3 Loss accounting

- `seq` increments per transmitted sample on each head unit (shared across ORIENT/RAW).
- The dongle tracks sequence continuity **per tracker ID** modulo 2¹⁶ and accumulates
  `seq_lost` per tracker.
- Duplicates (ACK lost, packet retransmitted and received twice) are detected by repeated
  per-tracker `seq` and dropped by the dongle — hosts never see duplicate sequence
  numbers from a given tracker.

### 2.4 Tracker identity

`id` is read once at boot from FICR `DEVICEADDR[0] & 0xFFFF` (remapped off the reserved
values as per §1.3). No pairing, provisioning, or persistence anywhere: a freshly flashed
board is immediately a valid tracker, and swapping hardware just makes a new ID appear in
the application's list.

### 2.5 Future (reserved, not in v1)

- **LAN bridge (`htbridge`)**: a host daemon that opens the dongle's serial port and
  republishes every frame (same payload format, without COBS) over UDP to the local
  network, plus a discovery beacon — so a WFS render cluster can see all trackers from
  one dongle. Planned after M6; purely additive, no changes to this protocol.
- Channel hopping on sustained loss (3-channel table; requires a resync rule).
- Pairing/whitening of addresses for RF-crowded or multi-rig environments (two dongles
  on the same channel would currently see each other's trackers — by design for now).
- Additional ESB pipes if shared-address collisions ever become a measured problem.
