// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Commands posted from the GUI thread to the real-time control thread.  They
// are delivered through a mutex-guarded FIFO and consumed once per control
// tick, so the GUI never touches EtherCAT state directly.

#pragma once

#include "actuator_test/types.hpp"

#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace actuator_test::gui
{

/// Connect to the bus using the given device-config TOML and bring all joints
/// to a safe idle state.
struct ConnectCommand
{
    std::string config_path;
};

/// Idle every drive and tear the bus down.
struct DisconnectCommand
{
};

/// Continuous manual jog of a single joint at a velocity in deg/s.  Send a
/// velocity of 0 to stop.  The controller integrates and clamps to soft limits.
struct JogCommand
{
    std::size_t joint = 0;
    double velocity_deg_s = 0.0;
};

/// Smoothly drive one joint to an absolute target angle (used by homing and
/// "go to centre").  Clamped to soft limits.
struct GoToCommand
{
    std::size_t joint = 0;
    double target_deg = 0.0;
    double speed_deg_s = 0.0; ///< <=0 uses the profile default approach speed.
};

/// Begin or end a backdrive limit-capture session for a set of joints.  While
/// active the controller idles the drives and tracks the min/max travelled.
struct CaptureLimitsCommand
{
    std::vector<std::size_t> joints;
    bool start = true; ///< true = begin capture, false = finish and keep.
};

/// Override the soft limits of a joint explicitly (deg).
struct SetLimitsCommand
{
    std::size_t joint = 0;
    double min_deg = 0.0;
    double max_deg = 0.0;
};

/// Reset one joint's learned envelope to its current position.
struct ResetLimitsCommand
{
    std::size_t joint = 0;
};

/// Start playing a parametric trajectory across the selected joints.
struct StartTrajectoryCommand
{
    std::vector<std::size_t> joints;
    TrajectoryMode mode = TrajectoryMode::Sin;
    bool enable_logging = true;
};

/// Stop any running trajectory and return to hold/idle.
struct StopCommand
{
};

/// Pause a running trajectory and hold current joint positions while staying
/// engaged (no idle/disconnect required).
struct PauseCommand
{
};

/// Terminate the worker thread (application shutdown).
struct ShutdownCommand
{
};

using Command = std::variant<ConnectCommand, DisconnectCommand, JogCommand, GoToCommand, CaptureLimitsCommand,
                             SetLimitsCommand, ResetLimitsCommand, StartTrajectoryCommand, StopCommand, PauseCommand,
                             ShutdownCommand>;

} // namespace actuator_test::gui
