cmake_minimum_required(VERSION 3.12)
project(SoapyMipiSDR CXX)

# Tuned for the Cortex-A76 in the Pi 5. -flto inlines the filter code across
# translation units and -ffast-math lets NEON skip strict IEEE handling.
add_compile_options(-O3 -ftree-vectorize -funsafe-math-optimizations -ffast-math -flto)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    add_compile_options(-mcpu=cortex-a76 -mtune=cortex-a76)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPY_SDR REQUIRED SoapySDR)

message(STATUS "SoapySDR Include: ${SOAPY_SDR_INCLUDE_DIRS}")
message(STATUS "SoapySDR Libs: ${SOAPY_SDR_LIBRARIES}")

add_library(SoapyMipiSDR MODULE
    Registration.cpp
    MipiDevice.cpp
    NEON.cpp
)

target_compile_features(SoapyMipiSDR PRIVATE cxx_std_17)
target_include_directories(SoapyMipiSDR PRIVATE ${SOAPY_SDR_INCLUDE_DIRS} .)

target_link_libraries(SoapyMipiSDR PRIVATE ${SOAPY_SDR_LIBRARIES})
# Link-Time Optimization requires linker flags as well
target_link_options(SoapyMipiSDR PRIVATE -flto ${FFTW3F_LDFLAGS})

set_target_properties(SoapyMipiSDR PROPERTIES 
    OUTPUT_NAME "mipi"
    BUILD_WITH_INSTALL_RPATH FALSE
    SKIP_BUILD_RPATH TRUE
)

# SoapySDR.pc exposes libdir but not the module directory, so build the path
# the loader scans: <libdir>/SoapySDR/modules<major>.<minor>.
if (NOT SOAPY_SDR_MODULE_PATH)
    pkg_get_variable(SOAPY_SDR_LIBDIR SoapySDR libdir)
    string(REGEX MATCH "^[0-9]+\\.[0-9]+" SOAPY_SDR_ABI "${SOAPY_SDR_VERSION}")
    if (NOT SOAPY_SDR_LIBDIR OR NOT SOAPY_SDR_ABI)
        message(FATAL_ERROR "cannot derive the SoapySDR module directory")
    endif()
    set(SOAPY_SDR_MODULE_PATH "${SOAPY_SDR_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI}")
endif()

message(STATUS "Installing to: ${SOAPY_SDR_MODULE_PATH}")
install(TARGETS SoapyMipiSDR DESTINATION ${SOAPY_SDR_MODULE_PATH})