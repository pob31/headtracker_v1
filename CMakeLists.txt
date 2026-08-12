# Top-level CMake entry: builds the HOST side only (client library, tools, tests).
# Firmware builds via west/Zephyr (see README), never through this file.
#
# Consumer apps (WFS-DIY / XOA / Tight-WFS) integrate via FetchContent or
# add_subdirectory of this repo and link `headtracker::client`; tests and tools
# stay off unless this project is built top-level.
#
# SPDX-License-Identifier: GPL-3.0-or-later
cmake_minimum_required(VERSION 3.20)
project(headtracker VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  set(HTK_TOP_LEVEL ON)
else()
  set(HTK_TOP_LEVEL OFF)
endif()

option(HTK_BUILD_TESTS "Build headtracker tests" ${HTK_TOP_LEVEL})
option(HTK_BUILD_TOOLS "Build headtracker tools (htgen)" ${HTK_TOP_LEVEL})
option(HTK_SANITIZE "Build library + tests with AddressSanitizer" OFF)
option(HTK_FUZZ "Build the libFuzzer target (needs clang or MSVC fuzzer support)" OFF)

if(HTK_BUILD_TESTS)
  enable_testing()
endif()

add_subdirectory(host)
