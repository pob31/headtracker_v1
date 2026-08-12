# SPDX-License-Identifier: GPL-3.0-or-later
add_executable(htgen htgen.cpp)
target_link_libraries(htgen PRIVATE headtracker::client)
if(MSVC)
  target_compile_options(htgen PRIVATE /W4)
else()
  target_compile_options(htgen PRIVATE -Wall -Wextra)
endif()
