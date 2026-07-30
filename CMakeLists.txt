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
    src/motion/path.cpp
    src/motion/segment_motion.cpp
    src/motion/lookahead.cpp
)

target_include_directories(frcnc_motion PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_motion PRIVATE frcnc_flags)

# --------------------------------------------------------------- ipc layer ----
# Header-only: the lock-free primitives are templates. The library target
# exists so consumers get the include path and Threads.

add_library(frcnc_ipc INTERFACE)

target_include_directories(frcnc_ipc INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_ipc INTERFACE Threads::Threads)

# ---------------------------------------------------------- fieldbus layer ----
# PDO offset discovery and DC drift compensation. Deliberately SOEM-free so the
# parts that can be silently wrong are testable without hardware.

add_library(frcnc_fieldbus STATIC
    src/fieldbus/pdo_map.cpp
    src/fieldbus/dc_sync.cpp
)

target_include_directories(frcnc_fieldbus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_fieldbus PRIVATE frcnc_flags)

# The SOEM-backed bus wrapper. Separate target so the SOEM-free parts above
# stay buildable and testable without the submodule.
if(FRCNC_WITH_SOEM)
    add_library(frcnc_bus STATIC
        src/fieldbus/bus.cpp
    )
    target_include_directories(frcnc_bus PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    # SOEM's headers are C and trip our stricter C++ warnings, so frcnc_flags
    # is deliberately not applied here.
    target_link_libraries(frcnc_bus PUBLIC frcnc_fieldbus soem)
endif()

# --------------------------------------------------------------- app layer ----
# Per-axis control and multi-axis coordination: user units in, drive counts out.
# Pure logic, so the safety rules -- above all the coordinated stop -- are
# testable without a drive on the bench.

add_library(frcnc_app STATIC
    src/app/axis_controller.cpp
    src/app/machine_controller.cpp
)

target_include_directories(frcnc_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(frcnc_app PUBLIC frcnc_drive frcnc_ipc PRIVATE frcnc_flags)

# The real-time thread that owns the bus and the machine. Separate target for
# the same reason as frcnc_bus: everything above stays buildable and testable
# without the SOEM submodule, and the coordinated stop is tested there.
if(FRCNC_WITH_SOEM)
    add_library(frcnc_task STATIC
        src/app/cyclic_task.cpp
    )
    target_include_directories(frcnc_task PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    )
    # SOEM's headers are C and trip our stricter C++ warnings, so frcnc_flags is
    # deliberately not applied here.
    target_link_libraries(frcnc_task PUBLIC frcnc_app frcnc_bus frcnc_rt)
endif()

# ------------------------------------------------------------------ tools ----

add_executable(rt_probe src/tools/rt_probe.cpp)
target_link_libraries(rt_probe PRIVATE frcnc_rt frcnc_flags)

if(FRCNC_WITH_SOEM)
    add_executable(bus_scan src/tools/bus_scan.cpp)
    # SOEM's headers are C and trip our stricter C++ warnings; keep frcnc_flags
    # off this target and rely on the compiler defaults.
    target_link_libraries(bus_scan PRIVATE frcnc_bus frcnc_drive)

    add_executable(bus_monitor src/tools/bus_monitor.cpp)
    target_link_libraries(bus_monitor PRIVATE frcnc_task)

    add_executable(axis_jog src/tools/axis_jog.cpp)
    target_link_libraries(axis_jog PRIVATE frcnc_task frcnc_motion)
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
