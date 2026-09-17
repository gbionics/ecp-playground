cmake_minimum_required(VERSION 3.28.3)

project(ecp-playground
    VERSION 0.0.1
    LANGUAGES CXX
    DESCRIPTION "GBionics SOEM Interface Primer")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_SCAN_FOR_MODULES OFF)

include(FetchContent)

FetchContent_Declare(
    ethercat_primer
    GIT_REPOSITORY  https://github.com/gbionics/ethercat-primer.git
    GIT_TAG         main
)

FetchContent_MakeAvailable(ethercat_primer)



# 1. Enable Qt automatic generation tools
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

option(BUILD_GUI "Build the Qt6 characterisation GUI" ON)

# Optional external DAQ verification (locked-rotor test): reads an
# independent torque sensor + encoder via NI-DAQmx. Only enabled when the
# driver is actually installed on this machine; see include/actuator_test/
# external_daq.hpp for the graceful no-op fallback otherwise.
#
# NOTE: pixi's bundled cmake (.pixi/envs/default/bin/cmake) has its own
# sysroot and does NOT search the host's standard multiarch library
# directories by default, so the NI-DAQmx .deb install (under
# /usr/lib/x86_64-linux-gnu, /usr/include) is invisible without explicit
# PATHS -- hence the hints below, rather than relying on the default search.
find_path(NIDAQMX_INCLUDE_DIR NAMES NIDAQmx.h
    PATHS /usr/include /usr/local/include)
find_library(NIDAQMX_LIBRARY NAMES nidaqmx
    PATHS /usr/lib/x86_64-linux-gnu /usr/lib /usr/local/lib)
if(NIDAQMX_INCLUDE_DIR AND NIDAQMX_LIBRARY)
    message(STATUS "NI-DAQmx found (${NIDAQMX_LIBRARY}): external DAQ torque/encoder verification enabled")
    set(ACTUATOR_TEST_HAVE_NIDAQMX TRUE)
else()
    message(STATUS "NI-DAQmx not found: external DAQ verification will be disabled (built as a no-op stub)")
    set(ACTUATOR_TEST_HAVE_NIDAQMX FALSE)
endif()

# Shared library of the reusable control/IO core (used by both the console tool
# and the GUI).  Keeping it in one place guarantees both front-ends drive the
# actuators through identical logic.
add_library(
    actuator_test_core STATIC
    src/actuator_test/device.cpp
    src/actuator_test/external_daq.cpp
    src/actuator_test/logging.cpp
    src/actuator_test/runtime.cpp
    src/actuator_test/safety.cpp
    src/actuator_test/session.cpp
    src/actuator_test/settings.cpp
    src/actuator_test/trajectory.cpp
)
set_property(TARGET actuator_test_core PROPERTY CXX_SCAN_FOR_MODULES OFF)
set_target_properties(actuator_test_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
target_include_directories(actuator_test_core PUBLIC include)
target_link_libraries(actuator_test_core PUBLIC ethercat-primer::ethercat-primer)
if(ACTUATOR_TEST_HAVE_NIDAQMX)
    # Scope the NI-DAQmx header hint to external_daq.cpp only (not
    # target-wide): pixi's conda-forge cross compiler ships its own glibc
    # headers, and -isystem would put /usr/include ahead of them, breaking
    # unrelated standard includes (e.g. <atomic> resolving to the host's
    # incompatible features.h). -idirafter only falls back to /usr/include
    # for headers conda's own sysroot doesn't provide, i.e. just NIDAQmx.h.
    set_source_files_properties(src/actuator_test/external_daq.cpp PROPERTIES
        COMPILE_OPTIONS "-idirafter;${NIDAQMX_INCLUDE_DIR}")
    target_link_libraries(actuator_test_core PRIVATE ${NIDAQMX_LIBRARY})
    target_compile_definitions(actuator_test_core PRIVATE ACTUATOR_TEST_HAVE_NIDAQMX)
endif()

add_executable(
    actuator-test-spline
    src/actuator-test-spline.cpp
)
set_property(TARGET actuator-test-spline PROPERTY CXX_SCAN_FOR_MODULES OFF)
set_target_properties(actuator-test-spline PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
target_link_libraries(actuator-test-spline PRIVATE actuator_test_core)

add_executable(
    actuator-cli
    src/actuator-cli.cpp
)
target_link_libraries(actuator-cli PRIVATE ethercat-primer::ethercat-primer)
add_custom_target(
    actuator-cli-capabilities
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/apply-actuator-capabilities.sh $<TARGET_FILE:actuator-cli>
    DEPENDS actuator-cli
    USES_TERMINAL
    COMMENT "Applying Linux capabilities to actuator-cli"
)

if(BUILD_GUI)
    add_subdirectory(gui)
endif()