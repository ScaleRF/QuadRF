cmake_minimum_required(VERSION 3.12)
project(OpenSpaceDaemon CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-O3 -mcpu=cortex-a76 -mtune=cortex-a76 -ftree-vectorize -funsafe-math-optimizations -ffast-math -flto)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

add_executable(os_streamerd main.cpp NEON.cpp)
target_include_directories(os_streamerd PRIVATE ${ZMQ_INCLUDE_DIRS} .)
target_link_libraries(os_streamerd PRIVATE ${ZMQ_LIBRARIES} pthread)
target_link_options(os_streamerd PRIVATE "-flto")
install(TARGETS os_streamerd DESTINATION bin)
