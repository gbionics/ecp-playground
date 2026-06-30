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
    GIT_REPOSITORY  git@github.com:gbionics/ethercat-primer.git
    GIT_TAG         fix-novanta
)

FetchContent_MakeAvailable(ethercat_primer)


# Override add_executable so that every executable target defined from here on
# (including those in subdirectories such as gui/) has the real-time Linux
# capabilities (re)applied at *configure* time.  This runs even when the binary
# was already built in a previous run, so a plain re-configure is enough to
# refresh the capabilities on existing executables.
macro(add_executable name)
    # Create the real target first.
    _add_executable(${name} ${ARGN})

    # Skip alias/imported declarations - they have no on-disk binary to patch.
    if(NOT "ALIAS" IN_LIST ARGN AND NOT "IMPORTED" IN_LIST ARGN)
        set(_rt_exe_path "${CMAKE_CURRENT_BINARY_DIR}/${name}")
        if(EXISTS "${_rt_exe_path}")
            message(STATUS "Applying RT capabilities to ${_rt_exe_path}")
            execute_process(
                COMMAND "${CMAKE_SOURCE_DIR}/scripts/apply-rt-capabilities.sh" "${_rt_exe_path}"
            )
        endif()
        unset(_rt_exe_path)
    endif()
endmacro()


# Shared library of the reusable control/IO core used by the console tools.
add_library(
    actuator_test_core STATIC
    src/actuator_test/device.cpp
    src/actuator_test/logging.cpp
    src/actuator_test/runtime.cpp
    src/actuator_test/safety.cpp
    src/actuator_test/session.cpp
    src/actuator_test/settings.cpp
    src/actuator_test/trajectory.cpp
)
set_property(TARGET actuator_test_core PROPERTY CXX_SCAN_FOR_MODULES OFF)
target_include_directories(actuator_test_core PUBLIC include)
target_link_libraries(actuator_test_core PUBLIC ethercat-primer::ethercat-primer)

add_executable(
    actuator-test-spline
    src/actuator-test-spline.cpp
)
target_link_libraries(actuator-test-spline PRIVATE actuator_test_core)

add_executable(
    actuator-cli
    src/actuator-cli.cpp
)
target_link_libraries(actuator-cli PRIVATE ethercat-primer::ethercat-primer)
add_custom_target(
    actuator-cli-capabilities
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/apply-rt-capabilities.sh $<TARGET_FILE:actuator-cli>
    DEPENDS actuator-cli
    USES_TERMINAL
    COMMENT "Applying Linux capabilities to actuator-cli"
)