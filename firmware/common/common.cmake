# Shared code pulled into both firmware apps.
# Usage (from an app CMakeLists.txt, after find_package(Zephyr)):
#   set(HTK_WITH_FUSION ON)   # head unit only
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/common.cmake)
set(HTK_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR})

zephyr_library_named(htk_common)
zephyr_library_sources(
  ${HTK_COMMON_DIR}/protocol/htk_crc16.c
  ${HTK_COMMON_DIR}/protocol/htk_cobs.c
  ${HTK_COMMON_DIR}/protocol/htk_frame.c
)
zephyr_include_directories(${HTK_COMMON_DIR})
zephyr_include_directories(${HTK_COMMON_DIR}/protocol)
zephyr_include_directories(${HTK_COMMON_DIR}/radio)

if(HTK_WITH_FUSION)
  zephyr_library_sources(
    ${HTK_COMMON_DIR}/fusion/htk_fusion.cpp
    ${HTK_COMMON_DIR}/fusion/vqf/vqf.cpp
  )
  zephyr_include_directories(${HTK_COMMON_DIR}/fusion)
  # M4F: single-precision floats keep VQF entirely on the FPU.
  zephyr_compile_definitions(VQF_SINGLE_PRECISION)
endif()
