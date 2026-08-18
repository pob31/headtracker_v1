# Shared code pulled into both firmware apps.
# Usage (from an app CMakeLists.txt, after find_package(Zephyr) + project()):
#   set(HTK_WITH_FUSION ON)   # head unit only
#   include(${CMAKE_CURRENT_SOURCE_DIR}/../common/common.cmake)
#
# Sources are added straight to the `app` target: a separate zephyr_library
# registered from an include()d file was silently left off the final link
# line under NCS 3.3 sysbuild, so we don't use one.
set(HTK_COMMON_DIR ${CMAKE_CURRENT_LIST_DIR})

target_sources(app PRIVATE
  ${HTK_COMMON_DIR}/protocol/htk_crc16.c
  ${HTK_COMMON_DIR}/protocol/htk_cobs.c
  ${HTK_COMMON_DIR}/protocol/htk_frame.c
)
zephyr_include_directories(${HTK_COMMON_DIR})
zephyr_include_directories(${HTK_COMMON_DIR}/protocol)
zephyr_include_directories(${HTK_COMMON_DIR}/radio)

if(HTK_WITH_FUSION)
  target_sources(app PRIVATE
    ${HTK_COMMON_DIR}/fusion/htk_fusion.cpp
    ${HTK_COMMON_DIR}/fusion/vqf/vqf.cpp
    ${HTK_COMMON_DIR}/imu/htk_tapdet.c
  )
  zephyr_include_directories(${HTK_COMMON_DIR}/fusion)
  zephyr_include_directories(${HTK_COMMON_DIR}/imu)
  # M4F: single-precision floats keep VQF entirely on the FPU.
  zephyr_compile_definitions(VQF_SINGLE_PRECISION)
endif()
