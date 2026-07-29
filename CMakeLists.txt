# Copyright (c) 2026 Front Range CNC. BSD 3-Clause. See LICENSE.
#
# SOEM v2 requires CMake >= 3.28, so that is the floor for the whole project.

cmake_minimum_required(VERSION 3.28)

project(frcnc_ethercat
    VERSION 0.1.0
    DESCRIPTION "Industrial EtherCAT CNC controller"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

# ---------------------------------------------------------------- options ----

option(FRCNC_BUILD_TESTS "Build unit tests" ON)
option(FRCNC_WITH_SOEM   "Build components that require SOEM" OFF)

# ------------------------------------------------------------------ flags ----

add_library(frcnc_flags INTERFACE)

target_compile_options(frcnc_flags INTERFACE
    -Wall -Wextra -Wpedantic
    -Wshadow -Wconversion -Wsign-conversion
    -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
    -Wunused -Woverloaded-virtual -Wdouble-promotion
    $<$<CONFIG:RelWithDebInfo,Release>:-O2>
    # Keep frame pointers: profiling a real-time loop needs usable stacks.
    -fno-omit-frame-pointer
)

# The cyclic path must not throw or unwind — see docs/05-motion-architecture.md §2.
# Applied only to the rt target for now; widen as the RT boundary firms up.
add_library(frcnc_rt_flags INTERFACE)
target_compile_options(frcnc_rt_flags INTERFACE -fno-exceptions -fno-rtti)

# ------------------------------------------------------------------- deps ----

find_package(Threads REQUIRED)

if(FRCNC_WITH_SOEM)
    if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/extern/soem/CMakeLists.txt")
        message(FATAL_ERROR
            "FRCNC_WITH_SOEM is ON but extern/soem is empty.\n"
            "  git submodule add https://github.com/OpenEtherCATsociety/SOEM.git extern/soem\n"
            "  git submodule update --init --recursive\n"
            "NOTE: SOEM v2 is GPLv3-or-commercial. See docs/08-licensing.md before shipping.")
    endif()
    add_subdirectory(extern/soem)
    # SOEM exports a plain target named `soem` — no namespace, so not SOEM::soem.
endif()

# --------------------------------------------------------------- rt layer ----
# Deliberately dependency-free beyond libc/pthreads, so it stays auditable.

add_library(frcnc_rt STATIC
    src/rt/cyclic_task.cpp
)

target_include_directories(frcnc_rt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_rt
    PUBLIC  Threads::Threads
    PRIVATE frcnc_flags
)

# rt (real-time clock/timer) is needed for clock_nanosleep on older glibc.
target_link_libraries(frcnc_rt PUBLIC $<$<PLATFORM_ID:Linux>:rt>)

# ------------------------------------------------------------- drive layer ----
# CiA 402 state machine. Pure logic — no fieldbus, no I/O, no allocation — so
# it is exhaustively testable without hardware.

add_library(frcnc_drive STATIC
    src/drive/cia402.cpp
)

target_include_directories(frcnc_drive PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_drive PRIVATE frcnc_flags frcnc_rt_flags)

# ------------------------------------------------------------ motion layer ----
# Trajectory generation. Pure math -- no fieldbus, no I/O.

add_library(frcnc_motion STATIC
    src/motion/profile.cpp
    src/motion/scurve.cpp
)

target_include_directories(frcnc_motion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_motion PRIVATE frcnc_flags)

# ------------------------------------------------------------------ tools ----

add_executable(rt_probe src/tools/rt_probe.cpp)
target_link_libraries(rt_probe PRIVATE frcnc_rt frcnc_flags)

if(FRCNC_WITH_SOEM)
    add_executable(bus_scan src/tools/bus_scan.cpp)
    # SOEM's headers are C and trip our stricter C++ warnings; keep frcnc_flags
    # off this target and rely on the compiler defaults.
    target_link_libraries(bus_scan PRIVATE soem)
endif()

# ------------------------------------------------------------------ tests ----

if(FRCNC_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()

# ---------------------------------------------------------------- summary ----

message(STATUS "")
message(STATUS "frcnc_ethercat ${PROJECT_VERSION}")
message(STATUS "  build type : ${CMAKE_BUILD_TYPE}")
message(STATUS "  tests      : ${FRCNC_BUILD_TESTS}")
message(STATUS "  SOEM       : ${FRCNC_WITH_SOEM}")
message(STATUS "")
