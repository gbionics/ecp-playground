// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Telemetry data exchanged from the real-time control thread to the GUI.
//
// The control thread runs at 1 kHz and must never block on the GUI.  It writes
// the latest snapshot into a double-buffered, mutex-guarded slot; the GUI polls
// that slot at its own (much lower) repaint rate.  This decouples the jitter
// sensitive control loop from Qt's event loop entirely.

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace actuator_test::gui {

/// High-level state of the controller, surfaced to the GUI for enabling and
/// disabling actions.
enum class ControllerState {
  Disconnected, ///< No bus; nothing enumerated.
  Connected,    ///< Bus up, joints enumerated, everything idle.
  Jogging,      ///< One or more joints under manual jog/home control.
  Capturing,    ///< Backdrive limit capture in progress.
  Running,      ///< A trajectory is playing.
  Faulted,      ///< A safety violation tripped; drives idled.
};

const char *to_string(ControllerState state) noexcept;

/// CiA-402 device state machine representation
enum class CiA402State {
  NotReadyToSwitchOn,
  SwitchedOnDisabled,
  ReadyToSwitchOn,
  SwitchedOn,
  OperationEnabled,
  QuickStop,
  FaultReactionActive,
  Fault,
  Unknown,
};

const char *to_string(CiA402State state) noexcept;

/// Per-joint live state for one telemetry frame.  All angles are in degrees so
/// the GUI never needs to know about encoder resolutions.
struct JointTelemetry {
  std::string name;
  double reference_deg = 0.0;       ///< Filtered command sent to the drive.
  double actual_deg = 0.0;          ///< Measured position feedback.
  double error_deg = 0.0;           ///< reference - actual.
  double velocity_deg_s = 0.0;      ///< Measured velocity.
  double acceleration_deg_s2 = 0.0; ///< Measured acceleration.
  double ref_velocity_deg_s = 0.0;  ///< Commanded velocity (for visualization).
  double ref_acceleration_deg_s2 = 0.0; ///< Commanded acceleration.
  double min_limit_deg = 0.0;
  double max_limit_deg = 0.0;
  double soft_min_limit_deg = 0.0;
  double soft_max_limit_deg = 0.0;
  double following_error_deg = 0.0; ///< Filtered following error for diagnosis.
  int16_t motor_temp_c = -1;
  int16_t drive_temp_c = -1;
  uint16_t status = 0;
  uint16_t error_code = 0;
  bool fault = false;
  bool limit_violation = false;      ///< Position exceeds soft limits.
  bool hard_limit_violation = false; ///< Position exceeds hard limits.
  int phase = 0; ///< ControlPhase (0=hold,1=approach,2=trajectory).
  CiA402State cia402_state = CiA402State::NotReadyToSwitchOn;
  bool homed = false;          ///< Has homing procedure completed.
  bool homing_active = false;  ///< Homing in progress.
  double torque_percent = 0.0; ///< Current torque as % of rated.
  double power_watts = 0.0;    ///< Estimated power consumption.
};

/// One coherent snapshot of the whole controller at a point in time.
struct TelemetryFrame {
  double t_s = 0.0; ///< Seconds since the loop started.
  ControllerState state = ControllerState::Disconnected;
  std::vector<JointTelemetry> joints;
  std::string status_line; ///< Optional human-readable status / reason.
  bool recording = false;  ///< True while free-running CSV recording is active.
  double recording_elapsed_s = 0.0; ///< Seconds since recording started.
  std::string
      recording_dir; ///< Directory holding the current recording (if any).
};

} // namespace actuator_test::gui
