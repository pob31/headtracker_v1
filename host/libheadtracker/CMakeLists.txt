# headtracker::client — host client library.
# Compiles the SAME protocol C sources as the firmware (single source of truth).
# SPDX-License-Identifier: GPL-3.0-or-later
set(HTK_PROTO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../firmware/common/protocol)

add_library(headtracker_client STATIC
  src/parser.cpp
  src/commands.cpp
  src/orientation.cpp
  ${HTK_PROTO_DIR}/htk_crc16.c
  ${HTK_PROTO_DIR}/htk_cobs.c
  ${HTK_PROTO_DIR}/htk_frame.c
)
add_library(headtracker::client ALIAS headtracker_client)

target_include_directories(headtracker_client PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${HTK_PROTO_DIR}
)
target_compile_features(headtracker_client PUBLIC cxx_std_17)

if(MSVC)
  target_compile_options(headtracker_client PRIVATE /W4)
else()
  target_compile_options(headtracker_client PRIVATE -Wall -Wextra)
endif()

if(HTK_SANITIZE)
  if(MSVC)
    target_compile_options(headtracker_client PUBLIC /fsanitize=address /Zi)
  else()
    target_compile_options(headtracker_client PUBLIC -fsanitize=address,undefined -g)
    target_link_options(headtracker_client PUBLIC -fsanitize=address,undefined)
  endif()
endif()
