// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/external_daq.hpp"

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
#include <NIDAQmx.h>

#include <algorithm>
#include <cmath>
#include <vector>
#endif

#include <chrono>

namespace actuator_test {

bool external_daq_supported() noexcept {
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  return true;
#else
  return false;
#endif
}

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)

namespace {

std::string last_daqmx_error() {
  char buf[2048] = {};
  DAQmxGetExtendedErrorInfo(buf, sizeof(buf));
  return std::string(buf);
}

/// 2nd-order Butterworth low-pass (bilinear transform), equivalent to
/// scipy.signal.butter(2, cutoff, fs=rate) used by the Python reference tool.
class Biquad {
public:
  Biquad(double cutoff_hz, double sample_rate_hz) {
    const double wc = std::tan(M_PI * cutoff_hz / sample_rate_hz);
    const double k = wc * wc;
    const double sqrt2 = std::sqrt(2.0);
    const double norm = 1.0 / (1.0 + sqrt2 * wc + k);
    m_b0 = k * norm;
    m_b1 = 2.0 * m_b0;
    m_b2 = m_b0;
    m_a1 = 2.0 * (k - 1.0) * norm;
    m_a2 = (1.0 - sqrt2 * wc + k) * norm;
  }

  double process(double x) {
    const double y = m_b0 * x + m_z1;
    m_z1 = m_b1 * x - m_a1 * y + m_z2;
    m_z2 = m_b2 * x - m_a2 * y;
    return y;
  }

private:
  double m_b0 = 0.0, m_b1 = 0.0, m_b2 = 0.0, m_a1 = 0.0, m_a2 = 0.0;
  double m_z1 = 0.0, m_z2 = 0.0;
};

/// X4 quadrature transition table indexed by (previous_state << 2) |
/// current_state; mirrors QUADRATURE_TRANSITIONS in the Python reference.
constexpr int kQuadratureTransitions[16] = {0,  1, -1, 0, -1, 0, 0, 1,
                                            1, 0,  0, -1, 0, -1, 1, 0};

} // namespace

struct ExternalDaqReader::Impl {
  TaskHandle ai_task = nullptr;
  TaskHandle di_task = nullptr;
  Biquad torque_lpf;

  int prev_quad_state = -1;
  int64_t speed_window_count = 0;
  int64_t speed_window_samples = 0;
  double filtered_rpm = 0.0;

  explicit Impl(const ExternalDaqConfig &cfg)
      : torque_lpf(cfg.lpf_cutoff_hz, cfg.ai_sample_rate_hz) {}
};

#endif // ACTUATOR_TEST_HAVE_NIDAQMX

#if !defined(ACTUATOR_TEST_HAVE_NIDAQMX)
// Trivial definition so std::unique_ptr<Impl> has a complete type to destroy
// even when built without NI-DAQmx (start() always fails before one is ever
// allocated).
struct ExternalDaqReader::Impl {};
#endif

ExternalDaqReader::ExternalDaqReader(ExternalDaqConfig cfg)
    : m_cfg(std::move(cfg)) {}

ExternalDaqReader::~ExternalDaqReader() { stop(); }

bool ExternalDaqReader::start(std::string &error) {
#if !defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  error = "built without NI-DAQmx support (driver not found at configure time)";
  return false;
#else
  if (m_running.load()) {
    return true;
  }
  auto impl = std::make_unique<Impl>(m_cfg);

  int32 status = DAQmxCreateTask("", &impl->ai_task);
  if (status) {
    error = last_daqmx_error();
    return false;
  }
  status = DAQmxCreateAIVoltageChan(impl->ai_task, m_cfg.analog_channel.c_str(),
                                    "", DAQmx_Val_NRSE, -10.0, 10.0,
                                    DAQmx_Val_Volts, nullptr);
  if (status == 0) {
    status = DAQmxCfgSampClkTiming(
        impl->ai_task, "", m_cfg.ai_sample_rate_hz, DAQmx_Val_Rising,
        DAQmx_Val_ContSamps,
        static_cast<uInt64>(m_cfg.ai_sample_rate_hz * 4)); // ~4 s buffer
  }
  if (status) {
    error = last_daqmx_error();
    DAQmxClearTask(impl->ai_task);
    return false;
  }

  status = DAQmxCreateTask("", &impl->di_task);
  if (status) {
    error = last_daqmx_error();
    DAQmxClearTask(impl->ai_task);
    return false;
  }
  const std::string lines = m_cfg.digital_line_a + "," + m_cfg.digital_line_b;
  status = DAQmxCreateDIChan(impl->di_task, lines.c_str(), "",
                             DAQmx_Val_ChanPerLine);
  if (status == 0) {
    status = DAQmxCfgSampClkTiming(
        impl->di_task, "", m_cfg.di_sample_rate_hz, DAQmx_Val_Rising,
        DAQmx_Val_ContSamps,
        static_cast<uInt64>(m_cfg.di_sample_rate_hz * 4)); // ~4 s buffer
  }
  if (status) {
    error = last_daqmx_error();
    DAQmxClearTask(impl->ai_task);
    DAQmxClearTask(impl->di_task);
    return false;
  }

  if (DAQmxStartTask(impl->ai_task) || DAQmxStartTask(impl->di_task)) {
    error = last_daqmx_error();
    DAQmxClearTask(impl->ai_task);
    DAQmxClearTask(impl->di_task);
    return false;
  }

  m_impl = std::move(impl);
  m_stop_requested.store(false);
  m_running.store(true);
  {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_last_error.clear();
  }
  m_thread = std::thread([this] { run(); });
  return true;
#endif
}

void ExternalDaqReader::stop() {
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  m_stop_requested.store(true);
  if (m_thread.joinable()) {
    m_thread.join();
  }
  if (m_impl) {
    if (m_impl->ai_task) {
      DAQmxStopTask(m_impl->ai_task);
      DAQmxClearTask(m_impl->ai_task);
    }
    if (m_impl->di_task) {
      DAQmxStopTask(m_impl->di_task);
      DAQmxClearTask(m_impl->di_task);
    }
    m_impl.reset();
  }
  m_running.store(false);
#endif
}

ExternalDaqReader::Sample ExternalDaqReader::latest() const {
  std::lock_guard<std::mutex> lk(m_mutex);
  return m_latest;
}

std::string ExternalDaqReader::lastError() const {
  std::lock_guard<std::mutex> lk(m_mutex);
  return m_last_error;
}

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)

void ExternalDaqReader::run() {
  const auto t0 = std::chrono::steady_clock::now();
  // Both reads must consume the same time-worth of samples per iteration --
  // the loop's actual cadence is dictated by whichever read blocks longest.
  // Previously the AI chunk was sized for 20 ms while the DI chunk (and thus
  // the loop period) was 50 ms, so every iteration left a 30 ms backlog of
  // unread AI samples; over time that grew until the AI buffer overflowed
  // (DAQmx error -200279, "not able to keep up with the hardware
  // acquisition"). Sizing both chunks from the same window fixes the drift.
  const int32 ai_chunk = std::max<int32>(
      1, static_cast<int32>(m_cfg.ai_sample_rate_hz * m_cfg.speed_window_s));
  const int32 di_chunk = std::max<int32>(
      1, static_cast<int32>(m_cfg.di_sample_rate_hz * m_cfg.speed_window_s));
  std::vector<float64> ai_buf(static_cast<std::size_t>(ai_chunk));
  std::vector<uInt8> di_buf(static_cast<std::size_t>(di_chunk) * 2);
  const double counts_per_rev = m_cfg.encoder_ppr * 4.0;
  constexpr double kSpeedLpfCutoffHz = 2.0; // matches the Python reference.

  while (!m_stop_requested.load()) {
    int32 ai_read = 0;
    if (DAQmxReadAnalogF64(m_impl->ai_task, ai_chunk, 1.0,
                           DAQmx_Val_GroupByChannel, ai_buf.data(),
                           static_cast<uInt32>(ai_buf.size()), &ai_read,
                           nullptr)) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "analog read failed: " + last_daqmx_error();
      break;
    }

    double last_raw_v = 0.0;
    double last_filtered_v = 0.0;
    for (int32 i = 0; i < ai_read; ++i) {
      last_raw_v = ai_buf[static_cast<std::size_t>(i)];
      last_filtered_v = m_impl->torque_lpf.process(last_raw_v);
    }

    int32 di_read = 0;
    int32 bytes_per_sample = 0;
    if (DAQmxReadDigitalLines(m_impl->di_task, di_chunk, 1.0,
                              DAQmx_Val_GroupByChannel, di_buf.data(),
                              static_cast<uInt32>(di_buf.size()), &di_read,
                              &bytes_per_sample, nullptr)) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "digital read failed: " + last_daqmx_error();
      break;
    }

    for (int32 i = 0; i < di_read; ++i) {
      const int a = di_buf[static_cast<std::size_t>(i)] ? 1 : 0;
      const int b = di_buf[static_cast<std::size_t>(di_read + i)] ? 1 : 0;
      const int state = (a << 1) | b;
      if (m_impl->prev_quad_state >= 0 && state != m_impl->prev_quad_state) {
        m_impl->speed_window_count +=
            kQuadratureTransitions[static_cast<std::size_t>(
                (m_impl->prev_quad_state << 2) | state)];
      }
      m_impl->prev_quad_state = state;
      ++m_impl->speed_window_samples;
    }

    const double window_time_s =
        static_cast<double>(m_impl->speed_window_samples) /
        m_cfg.di_sample_rate_hz;
    if (window_time_s > 0.0) {
      const double raw_rpm = static_cast<double>(m_impl->speed_window_count) /
                             counts_per_rev / window_time_s * 60.0;
      const double rc = 1.0 / (2.0 * M_PI * kSpeedLpfCutoffHz);
      const double alpha = window_time_s / (rc + window_time_s);
      m_impl->filtered_rpm += alpha * (raw_rpm - m_impl->filtered_rpm);
      m_impl->speed_window_count = 0;
      m_impl->speed_window_samples = 0;
    }

    Sample s;
    s.t_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
               .count();
    const double sign = m_cfg.invert_torque_sign ? -1.0 : 1.0;
    s.torque_nm = sign * last_raw_v / m_cfg.torque_sensitivity_v_per_nm;
    s.filtered_torque_nm =
        sign * last_filtered_v / m_cfg.torque_sensitivity_v_per_nm;
    s.speed_rpm = m_impl->filtered_rpm;

    std::lock_guard<std::mutex> lk(m_mutex);
    m_latest = s;
  }
  m_running.store(false);
}

#endif // ACTUATOR_TEST_HAVE_NIDAQMX

} // namespace actuator_test
