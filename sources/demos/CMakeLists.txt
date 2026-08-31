cmake_minimum_required(VERSION 3.12)
project(QuadRF_Demos C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -flto)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-mcpu=cortex-a76 -mtune=cortex-a76)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPY_SDR REQUIRED SoapySDR)

# quadrf-rf-vision
add_executable(quadrf-rf-vision csi_sweep.c mongoose.c)
target_include_directories(quadrf-rf-vision PRIVATE ${CMAKE_SOURCE_DIR}/sources/fpga/drivers/csi)
target_link_libraries(quadrf-rf-vision PRIVATE fftw3f SDL2 m pthread)

# quadrf-psd
add_executable(quadrf-psd psd_switchable.c)
target_include_directories(quadrf-psd PRIVATE ${CMAKE_SOURCE_DIR}/sources/fpga/drivers/csi)
target_link_libraries(quadrf-psd PRIVATE fftw3f SDL2 m)

# quadrf-ntsc-demod
add_executable(quadrf-ntsc-demod ntsc_demod.cpp)
target_compile_options(quadrf-ntsc-demod PRIVATE -ffast-math)
target_include_directories(quadrf-ntsc-demod PRIVATE ${SOAPY_SDR_INCLUDE_DIRS})
target_link_libraries(quadrf-ntsc-demod PRIVATE ${SOAPY_SDR_LIBRARIES})

# Install rules (standard binaries)
install(TARGETS quadrf-rf-vision quadrf-psd quadrf-ntsc-demod DESTINATION bin)
