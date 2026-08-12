# SPDX-License-Identifier: GPL-3.0-or-later
add_subdirectory(libheadtracker)

if(HTK_BUILD_TOOLS)
  add_subdirectory(tools/htgen)
  add_subdirectory(tools/htmon)
endif()

if(HTK_BUILD_TESTS)
  add_subdirectory(tests)
endif()
