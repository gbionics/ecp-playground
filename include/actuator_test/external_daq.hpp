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

#include "actuator_test/sample_buffer.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace actuator_test {

/// Channel/rate configuration, defaulted to match the Python reference tool.
struct ExternalDaqConfig {
  std::string analog_channel = "Dev1/ai28";        ///< Torsiometer AI.
  std::string temperature_channel; ///< Optional differential AT20, in the same AI task.
  std::string digital_line_a = "Dev1/port0/line5"; ///< Encoder quadrature A.
  std::string digital_line_b = "Dev1/port0/line6"; ///< Encoder quadrature B.
  double torque_sensitivity_v_per_nm = 0.025;      ///< TS112: +-5V @ +-200Nm.
  double ai_sample_rate_hz = 50000.0;
  double di_sample_rate_hz = 50000.0;
  double lpf_cutoff_hz = 50.0;    ///< Torque low-pass cutoff.
  double encoder_ppr = 720.0;     ///< Encoder pulses per revolution.
  double publish_period_s = 0.01; ///< Read chunk / sample publication period.
  /// Sliding window for the encoder speed estimate. Longer than the publish
  /// period so the quadrature quantisation stays fine (~0.42 rpm at 50 ms).
  double speed_window_s = 0.05;
  // The sensor's own sign convention (e.g. CCW-positive) is independent of
  // the drive's current-command convention; set this to align the two
  // instead of inverting the commanded current (which flips direction, not
  // the relationship between the two signs).
  bool invert_torque_sign = false;
};

/// True if this build was linked against NI-DAQmx (driver present at
/// configure time). When false, ExternalDaqReader::start() always fails.
bool external_daq_supported() noexcept;

/// Independent, on-demand differential voltage input for a 0-10 V AT20.
class TemperatureDaqReader {
public:
  struct Sample {
    double voltage_v = 0.0;
    double temperature_c = 0.0;
    double steady_time_s = 0.0;
  };
  explicit TemperatureDaqReader(std::string channel = "Dev1/ai0");
  ~TemperatureDaqReader();
  bool start(std::string &error);
  void stop();
  bool running() const { return m_running.load(); }
  std::optional<Sample> latest() const;
  std::string lastError() const;
  static double temperatureC(double voltage_v) { return 15.0 * voltage_v; }

private:
  void run();
  std::string m_channel;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_stop{false};
  std::thread m_thread;
  mutable std::mutex m_mutex;
  std::optional<Sample> m_latest;
  std::string m_error;
  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

/// Owns the NI-DAQmx analog + digital tasks and a background acquisition
/// thread. All accessors are thread-safe.
class ExternalDaqReader {
public:
  struct Sample {
    double t_s = 0.0;
    double torque_nm = 0.0;          ///< Raw (unfiltered) torque.
    double filtered_torque_nm = 0.0; ///< Low-pass filtered torque.
    double speed_rpm = 0.0;          ///< Low-pass filtered encoder speed.
    double delivery_time_s = 0.0;    ///< Host elapsed time at publication.
    uint64_t sample_index = 0;       ///< End of the hardware sample block.
    double steady_time_s = 0.0;     ///< Host monotonic publication time.
    std::optional<TemperatureDaqReader::Sample> temperature = std::nullopt;
  };

  explicit ExternalDaqReader(ExternalDaqConfig cfg = {});
  ~ExternalDaqReader();

  ExternalDaqReader(const ExternalDaqReader &) = delete;
  ExternalDaqReader &operator=(const ExternalDaqReader &) = delete;

  /// Opens the NI-DAQmx tasks and starts the acquisition thread. Returns
  /// false (with `error` filled in) if unsupported or the hardware/task setup
  /// fails or rates/DSP parameters are invalid; safe to call repeatedly,
  /// including after a worker read failure. Uses negotiated hardware rates.
  /// Calls to start() and stop() must be serialized by the caller.
  bool start(std::string &error);

  /// Stops acquisition and releases the tasks. Safe to call even if not
  /// running.
  void stop();

  bool running() const noexcept { return m_running.load(); }

  /// Thread-safe: most recently acquired sample (reset to default on each
  /// successful start, until the first read completes).
  Sample latest() const;
  struct TorqueMean {
    double torque_nm = 0.0;
    double span_s = 0.0;
    uint64_t samples = 0;
  };
  /// Raw sample-weighted mean over whole acquisition blocks (up to 60 s).
  /// Throws for invalid windows. Partial startup windows report actual span.
  std::optional<TorqueMean> torqueMean(double window_s) const;
  SampleBuffer<Sample> &captureBuffer() { return m_capture; }
  double analogSampleRateHz() const { return m_cfg.ai_sample_rate_hz; }
  const std::string &temperatureChannel() const { return m_cfg.temperature_channel; }
  double publicationPeriodS() const {
    return std::max(1.0, std::floor(m_cfg.ai_sample_rate_hz *
                                  m_cfg.publish_period_s)) /
           m_cfg.ai_sample_rate_hz;
  }

  /// Subtract `offset_nm` from all subsequent torque readings. The supplied
  /// offset is in the reader's current sign convention, so passing the
  /// measured zero-torque mean makes future readings zero-referenced.
  bool tare(double offset_nm);

  /// Thread-safe: cumulative offset subtracted by tare() since construction.
  double tareOffsetNm() const;

  /// Thread-safe: worker failure reason and any task stop/clear errors.
  /// Empty after error-free acquisition/cleanup. Cleanup errors append to
  /// existing diagnostics; cleared by the next successful start().
  std::string lastError() const;

private:
  void run();

  ExternalDaqConfig m_cfg;
  std::atomic<bool> m_running{false};
  std::atomic<bool> m_stop_requested{false};
  std::thread m_thread;
  mutable std::mutex m_mutex;
  Sample m_latest;
  struct TorqueBlock {
    double end_s;
    double sum_nm;
    uint64_t count;
  };
  std::deque<TorqueBlock> m_torque_blocks;
  double m_tare_offset_nm = 0.0;
  std::string m_last_error;
  SampleBuffer<Sample> m_capture;

  struct Impl;
  std::unique_ptr<Impl> m_impl;
};

} // namespace actuator_test
