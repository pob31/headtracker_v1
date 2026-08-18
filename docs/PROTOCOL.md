# Head Tracker Protocol Specification

**Protocol version:** 1 · **Spec status:** draft v0.6 (frozen at milestone M6) · 2026-08-18

This document specifies two wire formats:

- **Part 1 — USB host protocol**: what a dongle speaks over USB CDC-ACM to the host
  application. This is the public interface; client libraries implement exactly this.
- **Part 2 (appendix) — ESB air protocol**: what travels between head units and dongles
  over the 2.4 GHz link. Internal, but specified here because the payloads are shared
  with Part 1 by design.

### System model

The radio layer is a **broadcast fabric, pairing-free in both directions**:

- **Any number of head units** transmit; every packet carries the tracker's stable
  hardware ID.
- **Any number of receivers** listen; each dongle receives all trackers in range and
  multiplexes them onto its USB stream. Nothing is ACKed on the data path, so receivers
  are entirely passive and mutually invisible on it.
- Receivers announce their **presence with periodic beacons**; head units that hear no
  beacon for a while go to standby and probe at low duty cycle until a receiver returns.
  Beacons also carry the (rare) sensor-directed commands.
- **State vs. computation**: a command that mutates sensor state (fusion/bias reset,
  mode, rate) affects what *every* receiver sees, so such commands are always addressed
  to one specific tracker ID — never broadcast. Per-listener computation such as
  **recentering (yaw zeroing) is deliberately NOT a wire command**: it is receiver-side
  math (§1.6) applied in the client library, so each application keeps its own reference
  frame and no shared sensor state is touched.

The audio application lists the available trackers (from `TRACKER_STAT`) and the user
selects one in-app — swapping a dead battery or a different headphone rig means picking
the new unit from the list, nothing more. A LAN bridge daemon that republishes all
trackers to multiple render machines is on the roadmap (§2.6); it introduces no changes
to this protocol.

The C definitions in `firmware/common/protocol/htk_protocol.h` (once created) are the
single source of truth compiled into both firmware and the host library; this document
and that header must never disagree.

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
  no provisioning. Values `0x0000` and `0xFFFF` are reserved (invalid target and
  simulator source respectively); a head unit whose hardware value collides with a
  reserved value maps itself to `0x0001`/`0xFFFE`.
- `t_us` timestamps are the **originating head unit's** microsecond clock, wrapping at
  2³² (~71.6 min). Clocks of different trackers are unrelated. Hosts needing wall-clock
  alignment map them via arrival times (see PRD latency method).
- **Coordinate frame** (right-handed): body **X forward** (nose), **Y left**, **Z up**.
  World frame: Z up (gravity-aligned); the X/Y ("front") reference is whatever the
  *listener-side* recenter chose (§1.6) — the sensor itself has an arbitrary yaw origin.
- **Quaternion**: Hamilton convention, order **w, x, y, z**, unit-norm, representing the
  rotation from body frame to world frame. Yaw = rotation about world Z; suggested Euler
  extraction for display: intrinsic Z-Y'-X'' (yaw-pitch-roll).
- **ORIENT yaw is NOT the integral of the RAW gyro stream.** The head unit's fusion
  applies bias estimation and (when enabled) a slew-limited rest yaw-hold that
  re-anchors the yaw origin while the unit is provably still (flag bit `REST`). A host
  running its own filter on RAW data will legitimately disagree with ORIENT yaw by the
  amount of cancelled drift.
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

flags: bit 0 `HW_FUSION` (0 = software VQF, 1 = LSM6DSV16X SFLP) · bit 1 `BIAS_OK`
(gyro bias estimate genuinely converged; hysteretic) · bit 2 `SIM` (synthetic data,
not from a sensor) · bit 3 `REST` (the head unit's drift gate is armed: it considers
itself at rest and, when its yaw-hold servo is enabled, is actively holding yaw — the
unit's own gate, stricter than the raw fusion filter's) · bit 4 `TAP` (double-tap
gesture: a **level** held for ~250 ms of frames so a lossy link cannot swallow it;
hosts edge-detect, and after any stream gap MUST seed the previous-state from the
first received sample — never assume 0, or a reconnect mid-hold fabricates a tap) ·
bits 5–7 reserved, 0. REST and TAP were added additively per §1.9 — older clients
mask them away.

Sent continuously at each tracker's active rate (default ~208 Hz) whenever the tracker's
mode includes quaternions. Streams from multiple trackers interleave on the USB pipe;
consumers filter by `id`.

#### `0x02 STATUS` — dongle health (9 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x02` |
| 1 | uptime_ms | u32 | dongle uptime |
| 5 | rx_rate | u16 | total air packets received in the last second (all trackers) |
| 7 | n_trackers | u8 | trackers currently live (seen within the last 3 s) |
| 8 | flags | u8 | bit 0 `SIM_ACTIVE` · rest 0 |

Sent at 1 Hz, and immediately in response to `GET_STATS`.

#### `0x05 TRACKER_STAT` — per-tracker health (20 bytes)

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x05` |
| 1 | id | u16 | tracker ID |
| 3 | age_ms | u32 | time since this dongle last received a packet from this tracker |
| 7 | rate | u16 | packets/s from this tracker over the last second, as seen here |
| 9 | seq_lost | u32 | cumulative lost packets (sequence gaps) since dongle boot, as seen here |
| 13 | vbat_mV | u16 | head-unit battery voltage (self-reported), 0 if unknown |
| 15 | fw_major | u8 | head-unit firmware version (self-reported) — lets a lab spot stale units in a mixed fleet |
| 16 | fw_minor | u8 | |
| 17 | fw_patch | u8 | |
| 18 | mode | u8 | tracker's current mode (0 = quat, 1 = raw, 2 = both, self-reported) |
| 19 | flags | u8 | bit 0 `LINK_UP` (packet within last 500 ms) · rest 0 |

Sent at 1 Hz **per known tracker**, and after `GET_STATS`. This is the packet an
application uses to build its tracker list: any `id` seen here (or in ORIENT) is
available for selection. Trackers silent for 30 s are dropped from the dongle's table and
stop appearing. Loss and rate are *as observed by this dongle* — another receiver may see
different numbers.

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

Sent by trackers whose mode includes raw. Raw mode exists both for fusion tuning and to
move the fusion computation to the receiver side entirely (see PRD §computation
placement); a host running its own filter consumes RAW and ignores ORIENT.

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

Sensor-directed commands mutate tracker state that **all receivers in range observe**, so
each must name one specific target: `id` MUST be a concrete tracker ID (`0x0000` and
`0xFFFF` are rejected). There are intentionally no broadcast state changes.

| type | name | body | action |
|---|---|---|---|
| `0x10` | HELLO | proto_ver:u8 — version the host implements | dongle replies `HELLO_RESP` |
| `0x11` | RESET_FUSION | id:u16 | target re-initializes its fusion filter and gyro-bias estimate (recovery action; orientation output restarts from accel-derived attitude) |
| `0x12` | SET_RATE | id:u16, hz:u16 | target clamps to nearest supported rate (52/104/208/416); actual rate observable via `TRACKER_STAT.rate` |
| `0x13` | SET_MODE | id:u16, mode:u8 (0 = quat, 1 = raw, 2 = both) | target switches streamed packet types |
| `0x14` | GET_STATS | — | dongle sends `STATUS` + one `TRACKER_STAT` per tracker immediately |
| `0x15` | IDENTIFY | id:u16 | target flashes its LED white for ~3 s, so a physical unit can be matched to a list entry. No effect on streaming (harmless to repeat, like all commands) |
| `0x1F` | SIM_MODE | on:u8 (0/1) | dongle-local: emit synthetic ORIENT as tracker `0xFFFF` (slow yaw sweep, `SIM` flag set), alongside any real trackers |

**Delivery semantics:** sensor-directed commands are carried inside the dongle's presence
beacons (§2.3) and repeated in several consecutive beacons; head units filter by `id` and
commands are idempotent, so duplicate delivery is harmless. Because head units listen for
beacons at a low duty cycle, command latency is bounded by the listen interval — up to
roughly a second, which is acceptable for what are rare configuration/recovery actions
(nothing latency-critical travels toward the sensor). Effect is observable in
`TRACKER_STAT` (`rate`, `mode`) or in the stream itself (`BIAS_OK` drops after a
RESET_FUSION, then re-converges). Commands issued for a tracker that is absent or in
standby are carried for a few beacon repetitions and then expire — re-issue after
`TRACKER_STAT` shows `LINK_UP` again. With multiple receivers, nothing prevents two hosts
from configuring the same tracker; last command wins.

### 1.6 Recentering and boresight (client-side, not wire commands)

Both operations define the listener's reference frame; they are per-listener state, so
they live in the receiver's client library, not on the sensor. Each captures a
reference on the user's action — but they apply on **different sides** of the
quaternion product, and the side matters:

- **Recenter (yaw-only)**: capture `y_ref = heading(q_c)` — the unit quaternion
  `(w, 0, 0, z)/‖(w, 0, 0, z)‖` built from the current sample `q_c`'s w and z
  components. Yaw is a rotation in the **world** frame, so its inverse applies on the
  **left**: `q' = y_ref⁻¹ ⊗ q`. Zeroes "front" while keeping pitch/roll
  gravity-referenced. This is the routine correction for 6DoF yaw drift.
- **Boresight (full-pose tare)**: capture `b_ref = q_c` and its heading
  `h = heading(b_ref)` while the wearer holds their head level and looks straight
  ahead. Two rules make this exact for all subsequent motion:
  1. The mounting offset composes on the **body** side of a body→world quaternion
     (`q_sensor = q_head ⊗ q_mount`), so the tare divides on the **right**:
     `q ⊗ b_ref⁻¹` is the rotation since capture in world coordinates. (A left-side
     tare would conjugate later rotations into the tilted mount axes — a true 30°
     yaw would read ≈28° under a 25° mount roll.)
  2. That rotation must then be **re-expressed in the capture-aligned frame** —
     the fusion's world X/Y axes are arbitrary (6DoF yaw origin), so conjugate by
     the captured heading: `q' = h⁻¹ ⊗ (q ⊗ b_ref⁻¹) ⊗ h`. Without this, a pure
     physical roll about the "forward" axis decomposes into a yaw/pitch/roll
     mixture whenever the fusion's yaw origin differs from the capture direction
     (found on the bench: board on its side read as a three-axis mess).
- **Composed** (boresight once per rig, recenter freely afterwards):
  `q' = y_ref⁻¹ ⊗ h⁻¹ ⊗ q ⊗ b_ref⁻¹ ⊗ h`, with `y_ref` maintained compositionally:
  each recenter left-multiplies the inverse heading of the currently corrected pose.
- **KNOWN LIMITATION — mount azimuth**: a single-pose tare (and auto-level below,
  which sees only gravity) cannot observe the mount's twist about the vertical
  (3 constraints, 4 unknowns). Pure yaw motion still reads exactly, but a nod under an
  azimuthally-twisted mount cross-couples into roll/yaw by ~sin(azimuth). "Exact for
  all subsequent motion" above therefore holds iff the mount azimuth is zero — key the
  clip mechanically so the sensor's X roughly faces forward. A motion-based (nod)
  calibration could resolve it and is future work.
- **Display note**: yaw/pitch/roll extraction (Z-Y'-X'') is inherently degenerate at
  pitch = ±90° (gimbal lock: yaw and roll collapse into one degree of freedom and the
  numbers jump, while the quaternion remains perfectly valid). Renderers MUST consume
  the quaternion; Euler angles are for human-readable display only.

**Auto-level (recommended over manual boresight)**: the reference library's
`htk::Stabilizer` estimates the mount's tilt continuously from the long-term average
of body-frame gravity `g_s = q⁻¹ ⊗ (0,0,-1) ⊗ q` — provably invariant under every
world-yaw manipulation (drift, recenter, the head unit's rest yaw-hold), so nothing
yaw-related can poison it. Accumulation is gated on a wear detector (a worn head shows
tilt micro-motion; a headphone parked on a desk is dead still and teaches nothing), and
every tilt refinement re-anchors the yaw reference in the twist metric so the output
never jumps. The double-tap gesture (TAP flag) then only ever needs to set *yaw* —
safe from any posture, which is the point: headphones resting in odd positions must
never corrupt the reference. Manual `boresight()` remains as a deliberate override.

The reference library implements exactly this (`htk::Recenterer` + `htk::Stabilizer`);
each application (or each machine in a future LAN-bridged cluster) keeps its own
references. Sensor orientation origin is arbitrary and yaw drifts slowly (6DoF);
re-running either procedure costs nothing to any other receiver.

### 1.7 Session behavior

1. Host opens the port, sends `HELLO`, checks `proto_ver` in `HELLO_RESP`
   (major-version equality required; this spec is version 1).
2. Dongle streams ORIENT from every tracker in range as soon as air packets arrive —
   streaming neither waits for HELLO nor for any selection; the handshake is for the
   host's benefit only. The dongle beacons its presence on the radio whenever it is
   powered, which is what keeps head units out of standby.
3. The application builds its tracker list from `TRACKER_STAT` (1 Hz per tracker) and
   lets the user pick; "selection" is purely client-side filtering by `id`.
4. `STATUS` arrives at 1 Hz regardless of mode.
5. Settings on trackers are volatile per sensor power-cycle; defaults (208 Hz, quat) are
   restored on tracker reboot.

### 1.8 Worked examples (byte-exact)

CRC values below are normative test vectors for implementations.

**RESET_FUSION, tracker `0x1234`** — payload `11 34 12`, CRC16 = `0x43ED`:

```
payload+crc : 11 34 12 ED 43
frame       : 06 11 34 12 ED 43 00
```

**SET_MODE, tracker `0x1234` → raw** — payload `13 34 12 01`, CRC16 = `0x68EE`:

```
payload+crc : 13 34 12 01 EE 68
frame       : 07 13 34 12 01 EE 68 00
```

**IDENTIFY, tracker `0x1234`** — payload `15 34 12`, CRC16 = `0x9F2D`:

```
payload+crc : 15 34 12 2D 9F
frame       : 06 15 34 12 2D 9F 00
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
flags `BIAS_OK`; CRC16 = `0xF95F`:

```
payload     : 01 34 12 2A 00 40 42 0F 00 00 00 80 3F 00 00 00 00 00 00 00 00 00 00
              00 00 02
frame       : 05 01 34 12 2A 04 40 42 0F 01 01 03 80 3F 01 01 01 01 01 01 01 01 01
              01 01 04 02 5F F9 00
```

**TRACKER_STAT** — tracker `0x1234`, age 5 ms, 208 pkt/s, 3 lost, vbat 4012 mV,
fw 0.1.0, mode quat, `LINK_UP`; CRC16 = `0xE925`:

```
payload     : 05 34 12 05 00 00 00 D0 00 03 00 00 00 AC 0F 00 01 00 00 01
frame       : 05 05 34 12 05 01 01 02 D0 02 03 01 01 03 AC 0F 02 01 01 04 01 25 E9 00
```

### 1.9 Versioning and the consumer compatibility contract

The protocol **promises** to consumer applications:

1. The byte layout and meaning of an existing packet type never changes within a
   protocol version. Breaking changes bump `proto_ver`.
2. Evolution is **additive**: new packet types use previously unassigned type codes;
   new semantics on existing packets use previously reserved flag/caps bits. Neither
   bumps the version.
3. Reserved fields and bits read as 0 until assigned.

A consumer (or its client library) **must**, in return:

1. Send `HELLO` after connecting and check `HELLO_RESP.proto_ver` for equality with
   the version it implements; refuse or warn on mismatch.
2. Skip unknown packet types silently (COBS framing makes every frame skippable
   without understanding it). An unknown type is *not* an error.
3. Mask flag/caps fields to the bits it knows before interpreting them
   (`htk::kKnownOrientFlags` etc. in the reference library).
4. Treat malformed frames (CRC/COBS/length) as line noise: count, resync, continue.

The reference implementation of this contract is `htk::Parser` +
`htk::compatible()` in `host/libheadtracker/` — consumers using it get all four
obligations for free. A dongle firmware update can therefore ship new packet types to
a fleet without breaking any deployed application.

Freeze: this document is frozen at milestone M6 as v1.0; until then this draft may
change without a bump.

---

## Part 2 — Appendix: ESB air protocol

Internal to the firmware; hosts never see this layer.

### 2.1 Radio configuration (constants in `firmware/common/radio/`)

| parameter | value |
|---|---|
| protocol | Nordic ESB, dynamic payload length, max 32 B |
| data rate | 2 Mbps |
| RF channel | 77 (2477 MHz — above Wi-Fi ch. 13; single fixed channel in v1) |
| uplink address | base `A9 5E 3C D7` + prefix `48` — head units → all receivers |
| downlink address | base `A9 5E 3C D7` + prefix `21` — receiver beacons → all head units |
| ESB CRC | 16-bit (radio-level, independent of the USB CRC) |
| ACK / retransmit | **none** — every packet is sent no-ack (broadcast) |

**Why no ACKs:** with more than one receiver in range, all of them would ACK the same
uplink packet simultaneously and collide. Instead both directions are pure broadcast:
head units transmit data packets no-ack on the uplink address; every receiver hears every
tracker; receivers are passive and mutually invisible on the data path. A lost sample is
simply lost — the next one, ~4.8 ms later, supersedes it, and there is no retransmission
jitter in the latency path at all. Loss is measured per receiver via sequence gaps.

Air-time budget: a 26-byte packet at 2 Mbps is ~150 µs; at 208 Hz each tracker occupies
~3% air time, so 2–4 concurrent trackers collide only occasionally (uncorrelated clocks),
which shows up as a fraction of a percent in `seq_lost`. Beyond ~4 trackers, reduce
per-tracker rates via `SET_RATE`.

### 2.2 Uplink payloads (head unit → all receivers)

Air payloads are the **same bytes as the USB payload** (type byte + body) with **no COBS
and no CRC16** — ESB's own 16-bit CRC and fixed packet boundaries make both redundant.
The dongle's forwarding job is therefore: prepend nothing, append CRC16, COBS-encode,
send. Every air payload already carries the tracker `id`.

- `ORIENT` (26 B) and/or `RAW` (22 B) at the active rate, per the tracker's mode.
- A compact tracker-status payload (vbat, firmware version, mode, rate, uptime) every
  second, merged by each dongle into its `TRACKER_STAT` bookkeeping.

### 2.3 Downlink: receiver beacons (dongle → all head units)

Every powered dongle transmits a **beacon** on the downlink address every
`T_beacon = 100 ms` (no-ack; two dongles' beacons may occasionally collide, which the
repetition absorbs):

| offset | field | type | description |
|---|---|---|---|
| 0 | type | u8 | `0x20 BEACON` |
| 1 | beacon_seq | u8 | increments per beacon |
| 2 | cmd | u8[] | optional: exactly one embedded command payload (type + body as in §1.5), absent when idle |

Beacons serve two purposes:

1. **Presence**: hearing any beacon tells a head unit that at least one receiver is
   listening. Head units duty-cycle their radio to RX on the downlink address for one
   beacon interval (~110 ms) periodically — nominally every 1 s while active, every 5 s
   in standby. Timing constants are firmware-tuned, not protocol-normative.
2. **Command carriage**: a sensor-directed command from the host is embedded in every
   beacon for `N_repeat = 30` beacons (3 s — enough to cover the active listen interval
   with margin). Head units act on commands whose `id` matches their own and ignore the
   rest; idempotency makes duplicates harmless. Only the latest pending command per
   target is kept.

### 2.4 Power states (head unit)

| state | condition | behavior |
|---|---|---|
| ACTIVE | beacon heard within `T_hold = 15 s` | stream at the active rate; listen for beacons every ~1 s |
| STANDBY | no beacon for `T_hold` | stop streaming; IMU to low-power/off; wake every ~5 s to listen one beacon window; µA-level average draw |
| (optional) DEEP | standby **and** motionless (LSM6DS3TR-C wake-on-motion) | extend listen interval further; any motion returns to STANDBY cadence |

Recovery from standby is bounded by the standby listen interval (~5 s to reappear once a
receiver powers up). All constants are firmware-tuned.

### 2.5 Loss accounting

- `seq` increments per transmitted sample on each head unit (shared across ORIENT/RAW).
- Each receiver independently tracks sequence continuity **per tracker ID** modulo 2¹⁶
  and accumulates its own `seq_lost`.
- Without ACKs there are no retransmissions, hence no duplicates: hosts never see a
  repeated per-tracker `seq`.

### 2.6 Future (reserved, not in v1)

- **LAN bridge (`htbridge`)**: a host daemon that opens a dongle's serial port and
  republishes every frame (same payload format, without COBS) over UDP to the local
  network, plus a discovery beacon — so a WFS render cluster can see all trackers from
  one dongle. Planned after M6 (M8); purely additive, no changes to this protocol.
- Channel hopping on sustained loss (multi-channel table; needs a resync rule).
- Address whitening / site codes for RF-crowded venues (today, all rigs on channel 77
  share one broadcast fabric — receivers seeing all trackers is a feature, but separate
  *installations* may eventually want isolation).
- Packing two RAW samples per air packet (fits 32 B... 2×17 B data does not — would need
  a trimmed layout) if 416 Hz receiver-side fusion is wanted without doubling packet rate.
