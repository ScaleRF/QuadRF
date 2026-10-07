cmake_minimum_required(VERSION 3.16)
project(quadrf_wifi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

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

# Core Wi-Fi RX demodulator & MAC decoder library (headless, no SDL2 needed)
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

# Test executable (source lives under tests/, which is gitignored)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/tests/test_wifi_dsp.cpp")
  add_executable(test_wifi_dsp
    tests/test_wifi_dsp.cpp
    wifi_tx.cpp
  )

  target_link_libraries(test_wifi_dsp PRIVATE
    quadrf_wifi_rx_core
  )
endif()

# Data-link GUI/TUN binary: keep packaging intact when SDL2 is present
find_package(SDL2)
if(SDL2_FOUND)
  add_executable(quadrf-data
    main.cpp
    wifi_tx.cpp
    wifi_tun.cpp
    wifi_gui.cpp
  )

  target_include_directories(quadrf-data PRIVATE
    ${SDL2_INCLUDE_DIRS}
  )

  target_link_libraries(quadrf-data PRIVATE
    quadrf_wifi_rx_core
    ${SDL2_LIBRARIES}
  )

  install(TARGETS quadrf-data DESTINATION bin)
endif()

install(TARGETS quadrf-wifi-rx DESTINATION bin)
