# Head Tracker v1 — Product Requirements

**Status:** approved for implementation · **Date:** 2026-08-12

## 1. Goal

A wireless head tracker that feeds head orientation to spatial audio renderers with low
enough latency and high enough rate that dynamic binaural / wave-field synthesis rendering
feels anchored: the virtual scene stays put when the listener turns their head.

Target applications (all first-party): **WFS-DIY** (almost complete), **XOA** (in
progress), **Tight-WFS** (not started). None of them should need device-specific code
beyond a small client library that reads a serial port.

## 2. System overview

Two Seeed XIAO nRF52840 Sense boards:

- **Head units** (battery- or cable-powered, head-worn): sample the onboard LSM6DS3TR-C
  IMU at 416 Hz, run 6DoF sensor fusion (VQF) on the MCU, and **broadcast** orientation
  quaternions at ~208 Hz over Nordic ESB (Enhanced ShockBurst), a proprietary low-latency
  2.4 GHz protocol, with no ACKs and no pairing. ESB was chosen over BLE deliberately:
  BLE's connection interval floor (7.5 ms) puts ~8–12 ms in the path; an ESB packet is on
  air for ~150 µs.
- **Dongles** (USB, on a host): every receiver in range passively hears every tracker,
  wraps the packets in a COBS-framed binary protocol, and forwards them over USB CDC-ACM
  (a driverless virtual serial port on Windows / Linux / macOS). Receivers announce
  themselves with periodic radio **beacons**: presence keeps head units out of standby,
  and the beacons carry the rare sensor-directed commands (fusion reset, rate/mode) —
  always addressed to one specific tracker, since sensor state changes are visible to
  every receiver. Recentering is *not* a sensor command: it is per-listener math applied
  in the client library.

The full wire format is specified in [PROTOCOL.md](PROTOCOL.md).

## 3. Requirements

### Functional

| # | Requirement |
|---|-------------|
| F1 | Stream head orientation as unit quaternions with sequence number and device timestamp. |
| F2 | Default update rate ~208 Hz; host-settable via `SET_RATE`. IMU sampled at 416 Hz so transmitted samples are at most ~2.4 ms old at capture. |
| F3 | **Recenter + boresight**: client-library functions. Recenter zeroes the yaw reference (current heading becomes "front"); boresight captures the full pose to absorb the sensor's mounting angle — essential when the same sensor is quickly re-clipped to different headphones/headbands. Per-listener by design — neither touches sensor state, so each receiver/app keeps its own references. |
| F4 | **Raw mode**: stream raw gyro/accel samples — for fusion tuning, and to allow fusion to run receiver-side instead of on the head unit (see §computation placement). |
| F5 | **Simulator mode**: dongle can emit synthetic orientation data on command, so host software can be developed and tested with no head unit (or no radio link) present. |
| F6 | Link health surfaced to the host: per-tracker packet-loss counts (sequence gaps, as seen by that receiver), update rate, battery voltage, and firmware version (to spot stale units in a mixed fleet), at ≥1 Hz. |
| F7 | Version/identity handshake so client apps can check protocol compatibility. |
| F8 | **Multi-tracker, pairing-free**: a dongle receives every head unit in range (2–4 concurrent at full rate) and multiplexes them all onto the USB stream, tagged with a stable per-device hardware ID. The application lists available trackers and the user selects in-app — no pairing when a battery dies or headphones are swapped. |
| F9 | **Multi-receiver broadcast**: any number of dongles listen simultaneously; all see all trackers, passively and independently. Consequence: sensor-directed commands (fusion reset, rate, mode) are always addressed to one specific tracker ID — a state change is visible to every receiver, so nothing mutates sensor state anonymously or broadcast-wide. |
| F10 | **Auto standby**: head units stream only while at least one receiver's presence beacon has been heard recently; otherwise they drop to a µA-level standby, probing for beacons at low duty cycle (optionally deeper sleep when also motionless, via the IMU's wake-on-motion). Recovery when a receiver appears: seconds. |
| F11 | **Findable, self-explaining units**: an addressed `IDENTIFY` command flashes a tracker's LED white so a physical unit can be matched to its list entry. The head unit's RGB LED speaks a small state language: green blink = streaming, slow amber = no receiver heard, red = low battery, white = identify. |

### Performance

| # | Requirement |
|---|-------------|
| P1 | Motion-to-USB latency ≤ 5 ms typical (sensor sampling + fusion + air + USB). |
| P2 | Orientation rate ≥ 200 Hz sustained, jitter well under one period. |
| P3 | Yaw drift (6DoF, no magnetometer): minimized by VQF's online gyro-bias estimation and rest detection; drift is expected and recentering is the accepted correction. Target: slow enough that recentering more than every few minutes feels unnecessary in seated use. |
| P4 | Radio robustness: pure broadcast, no ACKs or retransmissions — a lost sample is superseded ~4.8 ms later, and the latency path has zero retransmission jitter. Loss is measured per receiver via sequence gaps and reported, not hidden. |

### Platform / operations

| # | Requirement |
|---|-------------|
| O1 | Host support: Windows 10+, Linux, macOS. Dongle is standard CDC-ACM; no drivers. |
| O2 | Firmware flashable with **no debug probe**: UF2 drag-and-drop via the stock Adafruit bootloader (double-tap reset). |
| O3 | Reproducible builds: nRF Connect SDK pinned to v3.3.0, CLI-only workflow (`nrfutil sdk-manager`), no IDE dependence. |
| O4 | The IMU sits behind a driver interface (`imu_source`) so the LSM6DSV16X can replace the LSM6DS3TR-C without touching fusion consumers, radio, or protocol. |

### Computation placement

Orientation fusion can run on either side of the radio; both are first-class in the
protocol, and the choice trades power and load:

| placement | air traffic | head-unit load | notes |
|---|---|---|---|
| **Head unit (default)** — VQF on the MCU | quat @ 208 Hz (~26 B packets) | fusion math (small on an M4F; radio dominates power) | one canonical orientation shared by all receivers |
| **Receiver side** — head streams raw | raw @ 208–416 Hz (~22 B packets) | minimal compute, more radio at 416 Hz | each receiver runs its own filter (freedom to experiment; results may differ between receivers) |
| **IMU silicon (M7)** — LSM6DSV16X SFLP | quat @ 208 Hz | near-zero (hardware fusion) | lowest-power endpoint of the same interface |

Radio transmission, not computation, dominates the head unit's power budget, so on-head
fusion at 208 Hz is also the low-power default; raw mode is selected per tracker via
`SET_MODE` when receiver-side fusion or filter tuning is wanted.

## 4. Non-goals (v1)

- **Position tracking** — orientation only (3DoF rotation).
- **Magnetometer / absolute heading** — yaw is relative; recenter defines "front".
- **More than ~4 concurrent trackers at full rate** — the shared-address air protocol
  degrades gracefully (lower per-tracker rates) but is not designed for crowds.
- **Battery management / charging UX** — power the head unit however is convenient.
- **Encryption or installation isolation** — pairing-free broadcast is a feature
  (F8/F9); fixed radio address constants are acceptable for lab/studio use. Separate
  installations sharing channel 77 would see each other's trackers; site codes/whitening
  are future work if that ever matters.
- **Configuration persistence** — settings are per-session, re-applied by the host.
- **LAN distribution in v1** — planned as a host daemon (`htbridge`, milestone M8) that
  republishes all trackers over UDP for multi-machine render clusters (WFS); additive,
  requires no protocol changes.

## 5. Roadmap

| Milestone | Content | Hardware needed |
|-----------|---------|-----------------|
| M0 | Repo scaffold; NCS v3.3.0 installed; both firmware apps compile to `.uf2` | no |
| M1 | Shared protocol lib (`htk_protocol.h`, COBS, CRC16) + host parser + unit tests + `htgen` synthetic stream tool | no |
| M2 | Vendor VQF; desktop harness runs the exact target fusion code on synthetic IMU data | no |
| M3 | First flash; dongle SIM_MODE streams over real USB → validates entire host path | yes |
| M4 | Broadcast air link live (no-ack uplink); receiver beacons + presence/standby; addressed command path; per-receiver loss/rate stats | yes |
| M5 | IMU bring-up (raw mode first, then on-target VQF); client-side recenter end-to-end in `htmon` | yes |
| M6 | Latency measurement + tuning (IRQ priorities, timing); protocol spec finalized v1.0 | yes |
| M7 | LSM6DSV16X breakout via external I2C; hardware SFLP fusion backend (`hw_fusion` flag) | yes |
| M8 | `htbridge` LAN daemon: republish all trackers over UDP + discovery, for render clusters | yes |

Multi-tracker (F8) is not a separate milestone: the protocol carries tracker IDs from M1,
the dongle's tracker table arrives with M4, and a second head unit is just another flash
of `app_head`.

## 6. Key technical facts and risks

Verified against Nordic/Zephyr documentation and source (Aug 2026):

- Zephyr board target **`xiao_ble/nrf52840/sense`**; the LSM6DS3TR-C is already in the
  board devicetree (`st,lsm6dsl` driver, I2C addr 0x6A, INT1 on P0.11, sensor VDD gated by
  a P1.08 regulator with 3 ms startup).
- **UF2 output is on by default** for this board; app partition at 0x27000. MCUboot must
  stay off (UF2 + sysbuild is broken upstream, Zephyr #88802).
- **ESB** samples ship in the SDK (`samples/esb/esb_ptx` / `esb_prx`); per-packet
  `noack` transmission is supported, which is what the broadcast fabric uses in both
  directions (dongles alternate between listening on the uplink address and emitting
  short presence beacons on the downlink address). 32-byte dynamic payloads fit our
  26-byte packet.
- **LSM6DSV16X hardware fusion (SFLP)** is supported in-tree since Zephyr v4.1 (in NCS
  3.3), exposing game-rotation-vector quaternions — the M7 upgrade path is real.
- **VQF** (github.com/dlaidig/vqf) is MIT-licensed, single-file C++, with online gyro-bias
  estimation and rest detection, designed for magnetometer-free operation.

Risks to engineer around:

1. **ESB owns interrupt priorities 0–1**; all other peripherals must be ≥ 2, but nRF52
   devicetree defaults put most at 1. Overlay USBD (dongle) and TWIM (head unit) to
   priority ≥ 2, or expect radio CRC errors under I/O load.
2. **Windows CDC read latency**: naive serial reads add 1–15 ms of buffering. The client
   library must set `COMMTIMEOUTS` for immediate partial reads; the dongle writes one
   frame per USB transfer so the parser never waits across a 64-byte boundary.
3. **IMU state survives soft reboot** (powered by an always-on regulator): firmware must
   SW_RESET the sensor during init (known Zephyr issue #55892 class of bugs).
4. **Zephyr USB stack transition** (legacy `usb_device` vs new `usbd`): prefer the new
   stack under NCS 3.3; fall back to legacy CDC-ACM if it fights the ESB sample scaffolding.

## 7. Acceptance (v1 done)

Wearing the head unit, running `htmon` on any of the three OSes: orientation streams at
≥200 Hz with reported loss <1% at 2 m line-of-sight, client-side recenter works in-app,
measured motion-to-USB latency ≤5 ms typical, and yaw drift in seated use is slow enough
that the scene remains stable between occasional recenters. A target application
(WFS-DIY) consumes the stream via the client library with no device-specific code beyond
it. With two head units powered, both appear in the tracker list and switching between
them in the app is immediate, with no pairing step. With two dongles on two machines,
both independently see all trackers. Unplugging every dongle sends the head units into
standby within ~15 s; plugging one back in restores streaming within seconds.
