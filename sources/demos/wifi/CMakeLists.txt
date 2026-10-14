cmake_minimum_required(VERSION 3.16)
project(quadrf_wifi LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 99)

add_compile_options(-Wall -Wextra -O3 -ffast-math)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  add_compile_options(-mcpu=cortex-a72)
else()
  add_compile_options(-march=native)
endif()

find_package(PkgConfig REQUIRED)
find_package(Threads REQUIRED)
find_package(ZLIB REQUIRED)
find_library(FFTW3F_LIB fftw3f REQUIRED)

# Check SoapySDR
if(NOT TARGET SoapySDR)
  pkg_check_modules(SOAPY REQUIRED SoapySDR)
endif()

# Core Wi-Fi RX demodulator & MAC decoder library
add_library(quadrf_wifi_rx_core STATIC
  wifi_viterbi.cpp
  wifi_dsp.cpp
  wifi_mac.cpp
  wifi_radio.cpp
)

target_include_directories(quadrf_wifi_rx_core PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SOAPY_INCLUDE_DIRS}
)

target_link_libraries(quadrf_wifi_rx_core PUBLIC
  ${SOAPY_LIBRARIES}
  ${FFTW3F_LIB}
  Threads::Threads
  ZLIB::ZLIB
  m
)

# Standalone receiver executable
add_executable(quadrf-wifi-rx
  wifi_rx_main.cpp
)

target_link_libraries(quadrf-wifi-rx PRIVATE
  quadrf_wifi_rx_core
)

# Test executable
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_wifi_dsp.cpp")
  add_executable(test_wifi_dsp
    tests/test_wifi_dsp.cpp
    wifi_tx.cpp
  )

  target_link_libraries(test_wifi_dsp PRIVATE
    quadrf_wifi_rx_core
  )
endif()

# 802.11 OFDM Link Daemon & Embedded Web HUD
add_executable(quadrf-data
  main.cpp
  wifi_link.cpp
  wifi_server.cpp
  wifi_tx.cpp
  wifi_tun.cpp
  mongoose.c
)

target_compile_definitions(quadrf-data PRIVATE
  WIFI_WEB_DIR="/usr/share/quadrf-data/web"
  MG_ENABLE_LINES=1
  MG_ENABLE_LOG=0
)

target_include_directories(quadrf-data PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}
)

target_link_libraries(quadrf-data PRIVATE
  quadrf_wifi_rx_core
)

install(TARGETS quadrf-data quadrf-wifi-rx DESTINATION bin)
install(DIRECTORY web/ DESTINATION share/quadrf-data/web)
