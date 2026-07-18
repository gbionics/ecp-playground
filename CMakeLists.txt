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

# Shared library of the reusable control/IO core (used by both the console tool
# and the GUI).  Keeping it in one place guarantees both front-ends drive the
# actuators through identical logic.
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
set_target_properties(actuator_test_core PROPERTIES AUTOMOC OFF AUTOUIC OFF AUTORCC OFF)
target_include_directories(actuator_test_core PUBLIC include)
target_link_libraries(actuator_test_core PUBLIC ethercat-primer::ethercat-primer)

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