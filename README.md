# headtracker_v1

A low-latency wireless head tracker for binaural / spatial audio rendering.

Head-worn IMU units stream orientation quaternions over a proprietary 2.4 GHz radio link
to a USB dongle, which forwards them to a spatial audio application (WFS-DIY, XOA,
Tight-WFS) over a plain serial port. Designed for a motion-to-USB latency of a few
milliseconds — well inside the perceptual budget for dynamic binaural rendering.

**Pairing-free broadcast, both ways:** head units broadcast; every receiver in range
hears every tracker, each packet tagged with a stable hardware ID. The audio app shows
the list of live trackers and the user picks one in-app — swapping a dead battery or a
different headphone rig means picking the new unit from the list, nothing more. Receivers
announce their presence with periodic radio beacons; head units that hear no receiver
drop into µA-level standby automatically and wake within seconds when one appears.

```
 head-worn units (1..4)                USB dongles (1..n)          host PCs
┌──────────────────────────┐   ESB    ┌─────────────────┐  CDC   ┌─────────────────────┐
│ LSM6DS3TR-C ─► VQF fusion│ 2.4 GHz  │ ESB RX ─► framer│  USB   │ spatial audio app   │
│ 416 Hz IMU     (6DoF)    │ ═══════► │  + tracker table│ ─────► │ tracker list, user  │
│ XIAO nRF52840 Sense  ×N  │ ◄beacon─ │ XIAO nRF52840 ×n│ ◄───── │ picks; WFS-DIY/XOA/ │
└──────────────────────────┘ presence └─────────────────┘  cmds  │ Tight-WFS           │
   quaternions @ ~208 Hz,    + cmds                              └─────────────────────┘
   no-ack broadcast, tracker-ID tagged                   COBS-framed binary protocol
```

## Status

**Pre-hardware, software complete to the extent possible.** Hardware (2× Seeed XIAO
nRF52840 Sense) is on its way. The spec is written; both firmware apps **compile to
flashable UF2** (never run on hardware yet — `VERIFY`/`TODO` markers gate the
hardware-dependent parts); the host client library builds and passes its full test
suite (spec conformance vectors, parser robustness, fuzzing, ASan-clean).

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
nrfutil sdk-manager install v3.3.0     # SDK + toolchain, ~11 GB under C:\ncs
```

Builds run from inside the SDK's west workspace (`C:\ncs\v3.3.0`). Windows quirk: if the
repo lives on a different drive than the SDK, west's relative-path handling breaks —
create a junction on the SDK's drive once and build through it:

```
mkdir C:\htk 2>NUL & rmdir C:\htk    # skip if C:\htk is free
mklink /J C:\htk d:\dev\headtracker_v1

nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 --chdir C:\ncs\v3.3.0 -- west build -b xiao_ble/nrf52840/sense C:\htk\firmware\app_head -d C:\htk\build_head
nrfutil sdk-manager toolchain launch --ncs-version v3.3.0 --chdir C:\ncs\v3.3.0 -- west build -b xiao_ble/nrf52840/sense C:\htk\firmware\app_dongle -d C:\htk\build_dongle
```

Artifacts land in `build_*/app_*/zephyr/zephyr.uf2`. Both apps currently build at ~10%
flash / ~13% RAM.

### Flashing (no debug probe needed)

The XIAO ships with the Adafruit UF2 bootloader and Zephyr's board config already emits a
UF2 image (`CONFIG_BUILD_OUTPUT_UF2=y`, app at flash offset 0x27000):

1. Double-tap the reset button — the board mounts as a `XIAO-SENSE` USB drive.
2. Copy `build_*/zephyr/zephyr.uf2` onto it. The board reboots into the app.

Do **not** enable MCUboot/sysbuild — UF2 generation is broken with it (Zephyr #88802),
and we don't need it. A bad app flash is always recoverable via double-tap reset.

## Host side

The dongle enumerates as a standard USB CDC-ACM serial port — driverless on Windows 10+,
Linux, and macOS. Any language that can open a serial port can consume the stream. The
reference C++17 client library lives in `host/libheadtracker/`:

- `htk::Parser` — push parser: feed raw serial bytes, get typed packet callbacks.
  Garbage-proof (fuzz-tested, ASan-clean) and version-tolerant (unknown packet types
  skipped, see PROTOCOL.md §1.9). Compiles the same protocol C sources as the firmware.
- `htk::encode_*` — command frame builders (recenter targets validated at the API).
- `htk::Recenterer` — client-side recenter (yaw) + boresight (mounting tare), §1.6.

Consumer apps integrate via CMake and link `headtracker::client`:

```cmake
include(FetchContent)
FetchContent_Declare(headtracker
  GIT_REPOSITORY https://github.com/pob31/headtracker_v1 GIT_TAG main)
FetchContent_MakeAvailable(headtracker)
target_link_libraries(your_app PRIVATE headtracker::client)
```

Build and test the host side standalone (any C++17 compiler; VS2026 tested):

```
cmake -B build-host -S .
cmake --build build-host --config Debug
ctest --test-dir build-host -C Debug
```

`host/tools/htgen` generates synthetic dongle streams (optionally corrupted with
`--garbage <pct>`) for testing consumers without hardware. A live CLI monitor (`htmon`)
plus serial transport arrive with hardware bring-up; the `htbridge` LAN daemon for
multi-machine render clusters is roadmap item M8.

## License

[GPL v3](LICENSE). © 2026 the headtracker_v1 contributors.
