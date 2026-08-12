# Head unit firmware — Seeed XIAO nRF52840 Sense (board: xiao_ble/nrf52840/sense)
# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(htk_app_head LANGUAGES C CXX)

# Pull in shared protocol/radio/fusion code (VQF needs C++, see prj.conf).
set(HTK_WITH_FUSION ON)
include(${CMAKE_CURRENT_SOURCE_DIR}/../common/common.cmake)

target_sources(app PRIVATE
  src/main.c
  src/radio.c
  src/leds.c
)
