// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Optional, independent verification channel: reads an external torque
// sensor (analog input) and quadrature encoder (digital lines) via NI-DAQmx,
// separate from the EtherCAT drive's own current/torque telemetry. Mirrors
// the reference Python implementation in
// dual-motors-testbench-sandbox/scripts/session1/ni_daq_reader.py, so a
// locked-rotor sweep can cross-check the drive's reported torque against a
// physically independent measurement.
//
// Compiled as a no-op stub (external_daq_supported() == false, start() always
// fails) unless the NI-DAQmx C driver was found at configure time -- see the
// root CMakeLists.txt. This keeps the feature building cleanly on machines
// without the driver installed, while remaining fully functional on the real
// test bench.

#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace actuator_test {

/// Channel/rate configuration, defaulted to match the Python reference tool.
struct ExternalDaqConfig {
  std::string analog_channel = "Dev1/ai28";        ///< Torsiometer AI.
  std::string digital_line_a = "Dev1/port0/line5"; ///< Encoder quadrature A.
  std::string digital_line_b = "Dev1/port0/line6"; ///< Encoder quadrature B.
  double torque_sensitivity_v_per_nm = 0.025;      ///< TS112: +-5V @ +-200Nm.
  double ai_sample_rate_hz = 50000.0;
  double di_sample_rate_hz = 50000.0;
  double lpf_cutoff_hz = 50.0;   ///< Torque low-pass cutoff.
  double encoder_ppr = 720.0;    ///< Encoder pulses per revolution.
  double speed_window_s = 0.05;  ///< Speed estimation window.
  // The sensor's own sign convention (e.g. CCW-positive) is independent of
  // the drive's current-command convention; set this to align the two
  // instead of inverting the commanded current (which flips direction, not
  // the relationship between the two signs).
  bool invert_torque_sign = false;
};

/// True if this build was linked against NI-DAQmx (driver present at
/// configure time). When false, ExternalDaqReader::start() always fails.
bool external_daq_supported() noexcept;

/// Owns the NI-DAQmx analog + digital tasks and a background acquisition
/// thread. All accessors are thread-safe.
class ExternalDaqReader {
public:
  struct Sample {
    double t_s = 0.0;
    double torque_nm = 0.0;          ///< Raw (unfiltered) torque.
    double filtered_torque_nm = 0.0; ///< Low-pass filtered torque.
    double speed_rpm = 0.0;          ///< Low-pass filtered encoder speed.
  };

  explicit ExternalDaqReader(ExternalDaqConfig cfg = {});
  ~ExternalDaqReader();

  ExternalDaqReader(const ExternalDaqReader &) = delete;
  ExternalDaqReader &operator=(const ExternalDaqReader &) = delete;

  /// Opens the NI-DAQmx tasks and starts the acquisition thread. Returns
  /// false (with `error` filled in) if unsupported or the hardware/task setup
  /// fails; safe to call repeatedly.
  bool start(std::string &error);

  /// Stops acquisition and releases the tasks. Safe to call even if not
  /// running.
  void stop();

  bool running() const noexcept { return m_running.load(); }

  /// Thread-safe: most recently acquired sample (default-constructed until
  /// the first read completes).
  Sample latest() const;

  /// Thread-safe: reason the acquisition thread stopped on its own (e.g. a
  /// DAQmx read error), empty if it's still running or was stopped via
  /// stop(). Cleared by the next successful start().
  std::string lastError() const;

private:
  void run();

  ExternalDaqConfig m_cfg;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_stop_requested{false};
  std::thread m_thread;
  mutable std::mutex m_mutex;
  Sample m_latest;
  std::string m_last_error;

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace actuator_test
