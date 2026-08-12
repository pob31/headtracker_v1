# headtracker_v1

A low-latency wireless head tracker for binaural / spatial audio rendering.

Head-worn IMU units stream orientation quaternions over a proprietary 2.4 GHz radio link
to a USB dongle, which forwards them to a spatial audio application (WFS-DIY, XOA,
Tight-WFS) over a plain serial port. Designed for a motion-to-USB latency of a few
milliseconds — well inside the perceptual budget for dynamic binaural rendering.

**Pairing-free multi-tracker:** the dongle receives every head unit in range and forwards
them all, each tagged with a stable hardware ID. The audio app shows the list of live
trackers and the user picks one in-app — swapping a dead battery or a different headphone
rig means picking the new unit from the list, nothing more.

```
 head-worn units (1..4)                USB dongle                  host PC
┌──────────────────────────┐   ESB    ┌─────────────────┐  CDC   ┌─────────────────────┐
│ LSM6DS3TR-C ─► VQF fusion│ 2.4 GHz  │ ESB RX ─► framer│  USB   │ spatial audio app   │
│ 416 Hz IMU     (6DoF)    │ ═══════► │  + tracker table│ ─────► │ tracker list, user  │
│ XIAO nRF52840 Sense  ×N  │ ◄─ ACK ─ │ XIAO nRF52840   │ ◄───── │ picks; WFS-DIY/XOA/ │
└──────────────────────────┘ commands └─────────────────┘  cmds  │ Tight-WFS           │
        quaternions @ ~208 Hz                                    └─────────────────────┘
        each tagged with tracker ID                      COBS-framed binary protocol
```

## Status

**Docs-first phase.** Hardware (2× Seeed XIAO nRF52840 Sense) is on its way. The product
requirements and the wire protocol are specified so that firmware and host software can be
developed independently, starting before the boards arrive.

- [docs/PRD.md](docs/PRD.md) — product requirements, architecture, roadmap
- [docs/PROTOCOL.md](docs/PROTOCOL.md) — byte-exact USB protocol spec + ESB air protocol

## Hardware

| Part | Role |
|------|------|
| Seeed XIAO nRF52840 Sense (×2) | One head-worn tracker, one USB receiver dongle |
| ST LSM6DS3TR-C (onboard) | 6-axis IMU, v1 sensor; software fusion (VQF) on the MCU |
| ST LSM6DSV16X (SparkFun breakout, later) | Upgrade IMU with hardware sensor fusion (SFLP) |

Datasheets live in [documentation/](documentation/).

## Planned repository layout

```
firmware/
  app_head/        # head-worn unit: IMU → fusion → ESB TX (PTX)
  app_dongle/      # USB dongle: ESB RX (PRX) → COBS framing → USB CDC
  common/
    protocol/      # htk_protocol.h — single source of truth, compiled by firmware AND host
    radio/         # ESB addresses, channel, pipe config, link stats
    imu/           # IMU source interface + LSM6DS3TR-C / LSM6DSV16X backends
    fusion/        # vendored VQF (MIT) + Mahony fallback, recenter/bias logic
host/
  libheadtracker/  # C++17 client library: pure push-parser core + serial transport
  tools/htmon/     # CLI monitor: tracker list, live Euler/quat, rates, loss, recenter key
  tools/htbridge/  # (M8) LAN daemon: republish all trackers over UDP for render clusters
docs/              # PRD, protocol spec
documentation/     # chip datasheets (PDF)
```

## Toolchain

Firmware targets the **nRF Connect SDK v3.3.0** (Zephyr), board target
`xiao_ble/nrf52840/sense`. Install on Windows without any IDE dependence:

```
nrfutil install sdk-manager
nrfutil sdk-manager toolchain install --ncs-version v3.3.0
nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 --shell
west build -b xiao_ble/nrf52840/sense firmware/app_head -d build_head
```

### Flashing (no debug probe needed)

The XIAO ships with the Adafruit UF2 bootloader and Zephyr's board config already emits a
UF2 image (`CONFIG_BUILD_OUTPUT_UF2=y`, app at flash offset 0x27000):

1. Double-tap the reset button — the board mounts as a `XIAO-SENSE` USB drive.
2. Copy `build_*/zephyr/zephyr.uf2` onto it. The board reboots into the app.

Do **not** enable MCUboot/sysbuild — UF2 generation is broken with it (Zephyr #88802),
and we don't need it. A bad app flash is always recoverable via double-tap reset.

## Host side

The dongle enumerates as a standard USB CDC-ACM serial port — driverless on Windows 10+,
Linux, and macOS. Any language that can open a serial port can consume the stream; a C++
reference library and CLI monitor are planned under `host/`. For multi-machine render
clusters, a LAN bridge daemon (`htbridge`) that republishes all trackers over UDP is on
the roadmap (M8).

## License

[GPL v3](LICENSE). © 2026 the headtracker_v1 contributors.
