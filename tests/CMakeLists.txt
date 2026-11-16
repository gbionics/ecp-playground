# SPDX-FileCopyrightText: Generative Bionics S.R.L.
# SPDX-License-Identifier: BSD-3-Clause

find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(Threads REQUIRED)

function(configure_probe target)
    set_target_properties(${target} PROPERTIES
        AUTOMOC OFF AUTOUIC OFF AUTORCC OFF CXX_SCAN_FOR_MODULES OFF)
    target_include_directories(${target} PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(${target} PRIVATE Threads::Threads)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX /UNDEBUG)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -UNDEBUG)
    endif()
endfunction()

function(add_probe_test name target runner environment)
    add_test(NAME ${name}
        COMMAND ${CMAKE_COMMAND} -E env
            "${environment}=$<TARGET_FILE:${target}>"
            "ACTUATOR_TEST_SCRATCH_DIR=${CMAKE_CURRENT_BINARY_DIR}"
            ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/${runner})
    set_tests_properties(${name} PROPERTIES TIMEOUT 180 LABELS "hardware-free")
endfunction()

function(enable_probe_ubsan target)
    if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
        target_compile_options(${target} PRIVATE
            -fsanitize=undefined -fno-sanitize-recover=undefined)
        target_link_options(${target} PRIVATE
            -fsanitize=undefined -fno-sanitize-recover=undefined)
    endif()
endfunction()

add_executable(speed-hold-probe speed_hold_probe.cpp)
configure_probe(speed-hold-probe)
enable_probe_ubsan(speed-hold-probe)
add_probe_test(speed-hold speed-hold-probe test_speed_hold.py ACTUATOR_TEST_SPEED_HOLD_PROBE)

add_executable(cyclic-current-probe cyclic_current_probe.cpp)
configure_probe(cyclic-current-probe)
enable_probe_ubsan(cyclic-current-probe)
add_test(NAME cyclic-current COMMAND $<TARGET_FILE:cyclic-current-probe>)
set_tests_properties(cyclic-current PROPERTIES TIMEOUT 10 LABELS "hardware-free")

add_executable(power-supply-probe
    power_supply_probe.cpp ${PROJECT_SOURCE_DIR}/src/actuator_test/power_supply.cpp)
configure_probe(power-supply-probe)
add_probe_test(power-supply power-supply-probe test_power_supply.py ACTUATOR_TEST_POWER_SUPPLY_PROBE)

foreach(mode IN ITEMS stub fake)
    set(target external-daq-${mode}-probe)
    add_executable(${target}
        external_daq_probe.cpp ${PROJECT_SOURCE_DIR}/src/actuator_test/external_daq.cpp)
    configure_probe(${target})
    enable_probe_ubsan(${target})
    if(mode STREQUAL "fake")
        target_compile_definitions(${target} PRIVATE ACTUATOR_TEST_HAVE_NIDAQMX)
        target_include_directories(${target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/external_daq_fake)
    endif()
    add_test(NAME external-daq-${mode} COMMAND $<TARGET_FILE:${target}>)
    set_tests_properties(external-daq-${mode} PROPERTIES TIMEOUT 10 LABELS "hardware-free")
endforeach()

if(BUILD_GUI)
    add_executable(thermal-csv-probe thermal_csv_probe.cpp)
    configure_probe(thermal-csv-probe)
    target_link_libraries(thermal-csv-probe PRIVATE actuator_test_gui)
    add_test(NAME thermal-csv COMMAND ${CMAKE_COMMAND} -E env
        "QT_QPA_PLATFORM=offscreen" $<TARGET_FILE:thermal-csv-probe>)
    set_tests_properties(thermal-csv PROPERTIES TIMEOUT 30 LABELS "hardware-free")

    add_executable(no-load-gui-probe no_load_gui_probe.cpp)
    configure_probe(no-load-gui-probe)
    target_link_libraries(no-load-gui-probe PRIVATE actuator_test_gui)
    add_probe_test(no-load-gui no-load-gui-probe test_no_load_gui.py ACTUATOR_TEST_NO_LOAD_GUI_PROBE)
endif()
