// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Finite ITECH IT-M3900C ELOG acquisition over LAN. Sampling is performed
// by the instrument; the worker downloads ASCII records after completion.
// Only remote mode and ELOG configuration are changed, never output settings.

#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace actuator_test {

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

private:
  void run();
  void connectSocket();
  void closeSocket();
  void command(const std::string &command);
  std::string query(const std::string &command, double timeout_s);
  void waitSocket(short events, std::chrono::steady_clock::time_point deadline);
  void setStatus(std::string status);
  void sleepFor(double seconds) const;

  PowerSupplyConfig m_cfg;
  std::atomic<bool> m_stop_requested{false};
  std::atomic<bool> m_cancel{false};
  std::atomic<bool> m_finished{false};
  std::atomic<bool> m_interrupted{false};
  std::thread m_thread;
  int m_fd = -1;

  mutable std::mutex m_mutex;
  std::vector<Sample> m_samples;
  std::string m_status;
};

} // namespace actuator_test
