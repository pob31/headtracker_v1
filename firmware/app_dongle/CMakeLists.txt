# SPDX-License-Identifier: GPL-3.0-or-later
# app_dongle — USB receiver dongle firmware (Seeed XIAO nRF52840 Sense).
# Build: west build -b xiao_ble/nrf52840/sense firmware/app_dongle
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(app_dongle LANGUAGES C)

# Shared protocol/framing code. HTK_WITH_FUSION stays off: the dongle never
# runs fusion — raw-mode consumers do that host-side.
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/common.cmake)

target_sources(app PRIVATE
  src/main.c
  src/leds.c
  src/usb_io.c
  src/radio.c
  src/trackers.c
  src/sim.c
)
