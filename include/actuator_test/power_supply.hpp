// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Finite ITECH IT-M3902C-80-40 ELOG acquisition over LAN. Sampling is performed
// by the instrument; the worker downloads ASCII records after completion.
// Only remote mode and ELOG configuration are changed, never output settings.
// PowerSupplyPoller instead polls the instrument's meter readings
// continuously (FETC:VOLT?/CURR?/POW?) for slow, open-ended acquisitions.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

namespace actuator_test {

namespace detail {

/// Minimal blocking-with-timeout SCPI-over-TCP client shared by the readers.
/// Every wait aborts promptly once `cancel` becomes true.
class ScpiSocket {
public:
  explicit ScpiSocket(const std::atomic<bool> &cancel) : m_cancel(cancel) {}
  ~ScpiSocket() { close(); }
  ScpiSocket(const ScpiSocket &) = delete;
  ScpiSocket &operator=(const ScpiSocket &) = delete;

  void connect(const std::string &host, uint16_t port, double timeout_s);
  void close();
  void command(const std::string &command, double timeout_s);
  std::string query(const std::string &command, double send_timeout_s,
                    double reply_timeout_s);

private:
  void wait(short events, std::chrono::steady_clock::time_point deadline);

  const std::atomic<bool> &m_cancel;
  int m_fd = -1;
};

/// Parses one SCPI numeric reply (e.g. "+1.234E+01").
double parseScpiNumber(std::string_view text);

} // namespace detail

struct PowerSupplyConfig {
  std::string host = "192.168.200.100";
  uint16_t port = 30000;
  double timeout_s = 2.0;
  double sample_period_s = 0.01;
  double duration_s = 30.0; ///< Must be finite and positive.
  double download_timeout_s = 30.0;
};

class PowerSupplyReader {
public:
  enum class State {
    Idle,
    Connecting,
    Arming,
    Acquiring,
    Downloading,
    Complete,
    Incomplete,
    Interrupted,
    Failed,
  };

  struct Progress {
    State state = State::Idle;
    std::string message;
    /// Retained after acquisition, so GUI polling cannot miss the trigger.
    bool acquisition_started = false;
  };

  struct Sample {
    double t_s = 0.0; ///< ELOG-relative time, not drive telemetry time.
    double voltage_v = 0.0;
    double current_a = 0.0;
    double power_w = 0.0; ///< voltage_v * current_a.
  };

  explicit PowerSupplyReader(PowerSupplyConfig cfg = {});
  ~PowerSupplyReader();

  PowerSupplyReader(const PowerSupplyReader &) = delete;
  PowerSupplyReader &operator=(const PowerSupplyReader &) = delete;

  /// Starts the background thread (non-blocking; connection happens there).
  void start();
  /// Non-blocking: abort ELOG and download the partial buffer.
  void requestStop();
  bool finished() const noexcept { return m_finished.load(); }
  /// Only call after finished(). Transfers the complete downloaded dataset.
  std::vector<Sample> takeSamples();
  bool interrupted() const noexcept { return m_interrupted.load(); }

  /// Human-readable connection state / last communication error.
  std::string status() const;
  /// Coherent state and display message; control flow must use the state.
  Progress progress() const;

private:
  void run();
  void command(const std::string &command);
  std::string query(const std::string &command, double timeout_s);
  void setProgress(State state, std::string message);
  void sleepFor(double seconds) const;

  PowerSupplyConfig m_cfg;
  std::atomic<bool> m_stop_requested{false};
  std::atomic<bool> m_cancel{false};
  std::atomic<bool> m_finished{false};
  std::atomic<bool> m_interrupted{false};
  detail::ScpiSocket m_socket{m_cancel};
  std::thread m_thread;

  mutable std::mutex m_mutex;
  std::vector<Sample> m_samples;
  Progress m_progress;
};

/// Continuously polls the ITECH meter readings at a fixed period until
/// stopped. Read-only: never enters remote mode and never touches output
/// settings, so the front panel stays usable during long thermal tests.
class PowerSupplyPoller {
public:
  struct Sample {
    double t_s = 0.0; ///< steadyNowS() at the midpoint of the queries.
    double voltage_v = 0.0;
    double current_a = 0.0;
    double power_w = 0.0;   ///< Instrument power reading (FETC:POW?).
    double latency_s = 0.0; ///< Round trip of the three queries.
  };

  PowerSupplyPoller(PowerSupplyConfig cfg, double period_s);
  ~PowerSupplyPoller();

  PowerSupplyPoller(const PowerSupplyPoller &) = delete;
  PowerSupplyPoller &operator=(const PowerSupplyPoller &) = delete;

  void start();
  void requestStop();
  /// The polling thread has exited (stopped or failed).
  bool finished() const noexcept { return m_finished.load(); }
  /// Samples acquired since the previous call.
  std::vector<Sample> takeNewSamples();
  std::string status() const;

  /// Monotonic clock in seconds, the time base of Sample::t_s.
  static double steadyNowS();

private:
  void run();
  void setStatus(std::string status);

  PowerSupplyConfig m_cfg;
  double m_period_s;
  std::atomic<bool> m_stop{false};
  std::atomic<bool> m_finished{false};
  detail::ScpiSocket m_socket{m_stop};
  std::thread m_thread;

  mutable std::mutex m_mutex;
  std::vector<Sample> m_new_samples;
  std::string m_status;
};

} // namespace actuator_test
