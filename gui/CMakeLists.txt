# SPDX-FileCopyrightText: Generative Bionics S.R.L.
# SPDX-License-Identifier: BSD-3-Clause
#
# Qt6 characterisation GUI.  Modular: each control panel is a self-contained
# widget, the real-time control lives in core/, and the window wires them up.

find_package(Qt6 REQUIRED COMPONENTS Widgets)

add_executable(
    actuator-test-gui
    main.cpp
    main_window.cpp
    core/controller_worker.cpp
    core/update_scheduler.cpp
    widgets/connection_panel.cpp
    widgets/jog_panel.cpp
    widgets/limits_panel.cpp
    widgets/trajectory_panel.cpp
    widgets/plot_panel.cpp
    widgets/drives_diagnostics_panel.cpp
    widgets/enhanced_limits_panel.cpp
    widgets/event_log_panel.cpp
    widgets/axis_overview_panel.cpp
    wizard/setup_wizard.cpp
    # Headers with Q_OBJECT listed so AUTOMOC always processes them.
    main_window.hpp
    core/controller_worker.hpp
    core/update_scheduler.hpp
    widgets/connection_panel.hpp
    widgets/jog_panel.hpp
    widgets/limits_panel.hpp
    widgets/trajectory_panel.hpp
    widgets/plot_panel.hpp
    widgets/drives_diagnostics_panel.hpp
    widgets/enhanced_limits_panel.hpp
    widgets/event_log_panel.hpp
    widgets/axis_overview_panel.hpp
    wizard/setup_wizard.hpp
)

set_property(TARGET actuator-test-gui PROPERTY CXX_SCAN_FOR_MODULES OFF)
set_target_properties(actuator-test-gui PROPERTIES AUTOMOC ON)

target_include_directories(actuator-test-gui PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(
    actuator-test-gui
    PRIVATE
        actuator_test_core
        Qt6::Widgets
)

add_custom_target(
    actuator-test-gui-capabilities
    COMMAND ${CMAKE_SOURCE_DIR}/scripts/apply-actuator-capabilities.sh $<TARGET_FILE:actuator-test-gui>
    DEPENDS actuator-test-gui
    USES_TERMINAL
    COMMENT "Applying Linux capabilities to actuator-test-gui"
)
