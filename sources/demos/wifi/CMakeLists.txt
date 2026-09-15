cmake_minimum_required(VERSION 3.16)
project(quadrf_wifi LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -O3 -ffast-math)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  add_compile_options(-mcpu=cortex-a76)
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

# Check SDL2
find_package(SDL2 REQUIRED)

add_library(quadrf_wifi_phy STATIC
  wifi_viterbi.cpp
  wifi_dsp.cpp
  wifi_tx.cpp
  wifi_mac.cpp
  wifi_tun.cpp
  wifi_radio.cpp
  wifi_gui.cpp
)

target_include_directories(quadrf_wifi_phy PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${SOAPY_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(quadrf_wifi_phy PUBLIC
  ${SOAPY_LIBRARIES}
  ${FFTW3F_LIB}
  ${SDL2_LIBRARIES}
  Threads::Threads
  ZLIB::ZLIB
  m
)

add_executable(quadrf-data
  main.cpp
)

target_link_libraries(quadrf-data PRIVATE
  quadrf_wifi_phy
)

install(TARGETS quadrf-data DESTINATION bin)
