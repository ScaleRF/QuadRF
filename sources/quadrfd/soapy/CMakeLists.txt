cmake_minimum_required(VERSION 3.12)
project(SoapyQuadRF_Thin CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(PkgConfig REQUIRED)
pkg_check_modules(SOAPY_SDR REQUIRED SoapySDR)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_library(SoapyQuadRF MODULE ThinSoapySDR.cpp)

target_include_directories(SoapyQuadRF PRIVATE ${SOAPY_SDR_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS} .)
target_link_libraries(SoapyQuadRF PRIVATE ${SOAPY_SDR_LIBRARIES} ${ZMQ_LIBRARIES})

set_target_properties(SoapyQuadRF PROPERTIES OUTPUT_NAME "quadrf" PREFIX "")

# Where the SoapySDR in use keeps its plugins. SoapySDR.pc exposes libdir but
# not the module directory, so build the path the loader scans.
if(NOT SOAPY_SDR_MODULE_PATH)
    pkg_get_variable(SOAPY_SDR_LIBDIR SoapySDR libdir)
    string(REGEX MATCH "^[0-9]+\\.[0-9]+" SOAPY_SDR_ABI "${SOAPY_SDR_VERSION}")
    if(NOT SOAPY_SDR_LIBDIR OR NOT SOAPY_SDR_ABI)
        message(FATAL_ERROR "cannot derive the SoapySDR module directory")
    endif()
    set(SOAPY_SDR_MODULE_PATH "${SOAPY_SDR_LIBDIR}/SoapySDR/modules${SOAPY_SDR_ABI}")
endif()

install(TARGETS SoapyQuadRF DESTINATION "${SOAPY_SDR_MODULE_PATH}")
