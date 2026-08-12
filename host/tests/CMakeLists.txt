# SPDX-License-Identifier: GPL-3.0-or-later
add_executable(htk_tests
  test_main.cpp
  test_vectors.cpp
  test_parser.cpp
  test_orientation.cpp
  test_fuzz_smoke.cpp
)
target_link_libraries(htk_tests PRIVATE headtracker::client)
target_include_directories(htk_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/vendor
                                             ${CMAKE_CURRENT_SOURCE_DIR})
if(MSVC)
  target_compile_options(htk_tests PRIVATE /W4)
else()
  target_compile_options(htk_tests PRIVATE -Wall -Wextra)
endif()
if(HTK_SANITIZE AND MSVC)
  target_compile_options(htk_tests PRIVATE /fsanitize=address /Zi)
endif()

add_test(NAME htk_tests COMMAND htk_tests)

# Optional coverage-guided fuzzer (clang libFuzzer / MSVC /fsanitize=fuzzer).
# Build: cmake -DHTK_FUZZ=ON ...; run: htk_fuzz_parser corpus\ -max_total_time=60
if(HTK_FUZZ)
  add_executable(htk_fuzz_parser fuzz/fuzz_parser.cpp)
  target_link_libraries(htk_fuzz_parser PRIVATE headtracker::client)
  target_include_directories(htk_fuzz_parser PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  if(MSVC)
    target_compile_options(htk_fuzz_parser PRIVATE /fsanitize=fuzzer /fsanitize=address /Zi)
  else()
    target_compile_options(htk_fuzz_parser PRIVATE -fsanitize=fuzzer,address -g)
    target_link_options(htk_fuzz_parser PRIVATE -fsanitize=fuzzer,address)
  endif()
endif()
