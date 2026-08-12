# SPDX-License-Identifier: GPL-3.0-or-later
add_executable(htmon htmon.cpp)
target_link_libraries(htmon PRIVATE headtracker::client)
if(MSVC)
  target_compile_options(htmon PRIVATE /W4)
else()
  target_compile_options(htmon PRIVATE -Wall -Wextra)
endif()
