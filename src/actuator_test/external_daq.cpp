// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/external_daq.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
#include <NIDAQmx.h>

#include <algorithm>
#include <deque>
#include <vector>
#endif

#include <chrono>

namespace actuator_test {

struct TemperatureDaqReader::Impl {
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  TaskHandle task = nullptr;
  bool started = false;
#endif
};

TemperatureDaqReader::TemperatureDaqReader(std::string channel)
    : m_channel(std::move(channel)) {}

TemperatureDaqReader::~TemperatureDaqReader() { stop(); }

std::optional<TemperatureDaqReader::Sample> TemperatureDaqReader::latest() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_latest;
}

std::string TemperatureDaqReader::lastError() const {
  std::lock_guard<std::mutex> lock(m_mutex);
  return m_error;
}

void TemperatureDaqReader::stop() {
  m_stop.store(true);
  if (m_thread.joinable())
    m_thread.join();
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  if (m_impl && m_impl->task) {
    const auto report = [this](int32 status) {
      if (status < 0) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_error += " NI-DAQmx task cleanup failed (code " +
                   std::to_string(status) + ")";
      }
    };
    if (m_impl->started)
      report(DAQmxStopTask(m_impl->task));
    report(DAQmxClearTask(m_impl->task));
  }
#endif
  m_impl.reset();
  m_running.store(false);
}

bool TemperatureDaqReader::start(std::string &error) {
  stop();
  {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_latest.reset();
    m_error.clear();
  }
#if !defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  error = "built without NI-DAQmx support";
  return false;
#else
  m_impl = std::make_unique<Impl>();
  const auto check = [&error](int32 status) {
    if (status >= 0)
      return true;
    char message[2048] = {};
    DAQmxGetExtendedErrorInfo(message, sizeof(message));
    error = message;
    return false;
  };
  if (!check(DAQmxCreateTask("", &m_impl->task)) ||
      !check(DAQmxCreateAIVoltageChan(
          m_impl->task, m_channel.c_str(), "", DAQmx_Val_Diff, -10.0, 10.0,
          DAQmx_Val_Volts, nullptr)) ||
      !check(DAQmxStartTask(m_impl->task))) {
    stop();
    return false;
  }
  m_impl->started = true;
  m_stop.store(false);
  m_running.store(true);
  try {
    m_thread = std::thread([this] { run(); });
  } catch (const std::exception &ex) {
    error = ex.what();
    stop();
    return false;
  }
  return true;
#endif
}

void TemperatureDaqReader::run() {
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  while (!m_stop.load()) {
    double volts = 0.0;
    int32 count = 0;
    const auto status = DAQmxReadAnalogF64(
        m_impl->task, 1, 0.25, DAQmx_Val_GroupByChannel, &volts, 1, &count, nullptr);
    if (status < 0 || count != 1 || !std::isfinite(volts)) {
      char message[2048] = {};
      if (status < 0)
        DAQmxGetExtendedErrorInfo(message, sizeof(message));
      std::lock_guard<std::mutex> lock(m_mutex);
      m_error = status < 0 ? std::string(message)
                          : "invalid or incomplete AT20 voltage sample";
      break;
    }
    const double now = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      m_latest = Sample{volts, temperatureC(volts), now};
    }
    for (int i = 0; i < 20 && !m_stop.load(); ++i)
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
#endif
  m_running.store(false);
}

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

void check_daqmx(int32 status) {
  if (status < 0)
    throw std::runtime_error(last_daqmx_error());
}

bool valid_config(const ExternalDaqConfig &cfg) {
  const auto positive = [](double value) {
    return std::isfinite(value) && value > 0.0;
  };
  return positive(cfg.ai_sample_rate_hz) && positive(cfg.di_sample_rate_hz) &&
         positive(cfg.publish_period_s) && positive(cfg.speed_window_s) &&
         positive(cfg.torque_sensitivity_v_per_nm) &&
         positive(cfg.encoder_ppr) && std::isfinite(cfg.encoder_ppr * 4.0) &&
         positive(cfg.lpf_cutoff_hz) &&
         cfg.lpf_cutoff_hz < cfg.ai_sample_rate_hz / 2.0 &&
         // Reject products outside the integer ranges used by NI and DSP.
         cfg.ai_sample_rate_hz * 4.0 < std::ldexp(1.0, 64) &&
         cfg.di_sample_rate_hz * 4.0 < std::ldexp(1.0, 64) &&
         cfg.ai_sample_rate_hz * cfg.publish_period_s < std::ldexp(1.0, 31) &&
         (cfg.temperature_channel.empty() ||
          cfg.ai_sample_rate_hz * cfg.publish_period_s < std::ldexp(1.0, 30)) &&
         // The two DI channels also use signed (di_read + i) indexing.
         cfg.di_sample_rate_hz * cfg.publish_period_s < std::ldexp(1.0, 30) &&
         // Leave room for the retained edge chunk and the next appended chunk.
         cfg.di_sample_rate_hz * cfg.speed_window_s <
             std::ldexp(1.0, 63) - std::ldexp(1.0, 31);
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
constexpr int kQuadratureTransitions[16] = {0, 1, -1, 0,  -1, 0,  0, 1,
                                            1, 0, 0,  -1, 0,  -1, 1, 0};

} // namespace

struct ExternalDaqReader::Impl {
  explicit Impl(ExternalDaqReader &reader) : owner(reader) {}
  ~Impl() {
    for (const auto task : {di_task, ai_task}) {
      if (!task)
        continue;
      if ((task == ai_task ? ai_started : di_started))
        cleanup(DAQmxStopTask(task), "DAQmxStopTask");
      cleanup(DAQmxClearTask(task), "DAQmxClearTask");
    }
  }
  void cleanup(int32 status, const char *operation) {
    if (status >= 0)
      return;
    const auto detail = last_daqmx_error();
    std::lock_guard<std::mutex> lock(owner.m_mutex);
    if (!owner.m_last_error.empty())
      owner.m_last_error += "; ";
    owner.m_last_error += std::string(operation) + " failed: " + detail;
  }

  ExternalDaqReader &owner;
  TaskHandle ai_task = nullptr, di_task = nullptr;
  bool ai_started = false, di_started = false;
};

#else
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
  // A failed worker is no longer running, but its thread is still joinable.
  stop();
  if (!valid_config(m_cfg)) {
    error = "invalid DAQ configuration (rates, DSP parameters or buffer sizes)";
    return false;
  }
  try {
    auto impl = std::make_unique<Impl>(*this);
    auto cfg = m_cfg;
    check_daqmx(DAQmxCreateTask("", &impl->ai_task));
    check_daqmx(DAQmxCreateAIVoltageChan(
        impl->ai_task, cfg.analog_channel.c_str(), "", DAQmx_Val_NRSE, -10.0,
        10.0, DAQmx_Val_Volts, nullptr));
    if (!cfg.temperature_channel.empty())
      check_daqmx(DAQmxCreateAIVoltageChan(
          impl->ai_task, cfg.temperature_channel.c_str(), "", DAQmx_Val_Diff,
          -10.0, 10.0, DAQmx_Val_Volts, nullptr));
    check_daqmx(DAQmxCfgSampClkTiming(
        impl->ai_task, "", cfg.ai_sample_rate_hz, DAQmx_Val_Rising,
        DAQmx_Val_ContSamps, static_cast<uInt64>(cfg.ai_sample_rate_hz * 4)));

    check_daqmx(DAQmxCreateTask("", &impl->di_task));
    const std::string lines = cfg.digital_line_a + "," + cfg.digital_line_b;
    check_daqmx(DAQmxCreateDIChan(impl->di_task, lines.c_str(), "",
                                  DAQmx_Val_ChanPerLine));
    check_daqmx(DAQmxCfgSampClkTiming(
        impl->di_task, "", cfg.di_sample_rate_hz, DAQmx_Val_Rising,
        DAQmx_Val_ContSamps, static_cast<uInt64>(cfg.di_sample_rate_hz * 4)));
    check_daqmx(DAQmxGetSampClkRate(impl->ai_task, &cfg.ai_sample_rate_hz));
    check_daqmx(DAQmxGetSampClkRate(impl->di_task, &cfg.di_sample_rate_hz));
    if (!valid_config(cfg))
      throw std::runtime_error(
          "invalid DAQ configuration for negotiated sample rates");
    check_daqmx(DAQmxStartTask(impl->ai_task));
    impl->ai_started = true;
    check_daqmx(DAQmxStartTask(impl->di_task));
    impl->di_started = true;

    m_cfg = std::move(cfg);
    m_impl = std::move(impl);
    m_stop_requested.store(false);
    m_running.store(true);
    {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_latest = {};
      m_torque_blocks.clear();
      m_last_error.clear();
    }
    m_thread = std::thread([this] { run(); });
  } catch (const std::exception &ex) {
    m_running.store(false);
    m_impl.reset();
    error = ex.what();
    return false;
  }
  return true;
#endif
}

void ExternalDaqReader::stop() {
#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  m_stop_requested.store(true);
  if (m_thread.joinable()) {
    m_thread.join();
  }
  m_impl.reset();
  m_running.store(false);
#endif
}

ExternalDaqReader::Sample ExternalDaqReader::latest() const {
  std::lock_guard<std::mutex> lk(m_mutex);
  return m_latest;
}

std::optional<ExternalDaqReader::TorqueMean>
ExternalDaqReader::torqueMean(double window_s) const {
  if (!std::isfinite(window_s) || window_s < 0.1 || window_s > 60.0)
    throw std::invalid_argument("torque averaging window must be 0.1-60 seconds");
  std::lock_guard<std::mutex> lk(m_mutex);
  if (m_torque_blocks.empty())
    return std::nullopt;
  TorqueMean mean;
  double sum = 0.0;
  const double target_samples = window_s * m_cfg.ai_sample_rate_hz;
  for (auto it = m_torque_blocks.rbegin(); it != m_torque_blocks.rend(); ++it) {
    sum += it->sum_nm;
    mean.samples += it->count;
    if (static_cast<double>(mean.samples) >= target_samples - 1e-7)
      break;
  }
  if (mean.samples == 0)
    return std::nullopt;
  mean.span_s = mean.samples / m_cfg.ai_sample_rate_hz;
  mean.torque_nm = sum / mean.samples - m_tare_offset_nm;
  return mean;
}

bool ExternalDaqReader::tare(double offset_nm) {
  if (!std::isfinite(offset_nm) || !m_running.load()) {
    return false;
  }
  std::lock_guard<std::mutex> lk(m_mutex);
  m_tare_offset_nm += offset_nm;
  m_latest.torque_nm -= offset_nm;
  m_latest.filtered_torque_nm -= offset_nm;
  return true;
}

double ExternalDaqReader::tareOffsetNm() const {
  std::lock_guard<std::mutex> lk(m_mutex);
  return m_tare_offset_nm;
}

std::string ExternalDaqReader::lastError() const {
  std::lock_guard<std::mutex> lk(m_mutex);
  return m_last_error;
}

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)

void ExternalDaqReader::run() {
  Biquad torque_lpf(m_cfg.lpf_cutoff_hz, m_cfg.ai_sample_rate_hz);
  int prev_quad_state = -1;
  int64_t speed_window_count = 0, speed_window_samples = 0;
  std::deque<std::pair<int64_t, int64_t>> speed_chunks;
  double filtered_rpm = 0.0;
  const auto t0 = std::chrono::steady_clock::now();
  // Consume equal time windows to avoid accumulating an AI or DI backlog.
  const int32 ai_chunk = std::max<int32>(
      1, static_cast<int32>(m_cfg.ai_sample_rate_hz * m_cfg.publish_period_s));
  const int32 di_chunk = std::max<int32>(
      1, static_cast<int32>(m_cfg.di_sample_rate_hz * m_cfg.publish_period_s));
  const bool has_temperature = !m_cfg.temperature_channel.empty();
  std::vector<float64> ai_buf(static_cast<std::size_t>(ai_chunk) *
                            (has_temperature ? 2 : 1));
  std::vector<uInt8> di_buf(static_cast<std::size_t>(di_chunk) * 2);
  const double counts_per_rev = m_cfg.encoder_ppr * 4.0;
  constexpr double kSpeedLpfCutoffHz = 2.0; // matches the Python reference.
  const auto window_samples =
      static_cast<int64_t>(m_cfg.di_sample_rate_hz * m_cfg.speed_window_s);
  const double chunk_time_s = di_chunk / m_cfg.di_sample_rate_hz;
  const double rc = 1.0 / (2.0 * M_PI * kSpeedLpfCutoffHz);
  const double alpha = chunk_time_s / (rc + chunk_time_s);
  uint64_t analog_samples = 0;

  while (!m_stop_requested.load()) {
    int32 ai_read = 0;
    if (DAQmxReadAnalogF64(m_impl->ai_task, ai_chunk, 1.0,
                           DAQmx_Val_GroupByChannel, ai_buf.data(),
                           static_cast<uInt32>(ai_buf.size()), &ai_read,
                           nullptr) < 0) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "analog read failed: " + last_daqmx_error();
      break;
    }
    if (ai_read != ai_chunk) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "incomplete analog block: expected " +
                     std::to_string(ai_chunk) + ", received " +
                     std::to_string(ai_read);
      break;
    }

    double last_filtered_v = 0.0, sum_raw_v = 0.0;
    for (int32 i = 0; i < ai_read; ++i) {
      const double voltage = ai_buf[static_cast<std::size_t>(i)];
      sum_raw_v += voltage;
      last_filtered_v = torque_lpf.process(voltage);
    }

    int32 di_read = 0;
    int32 bytes_per_sample = 0;
    if (DAQmxReadDigitalLines(m_impl->di_task, di_chunk, 1.0,
                              DAQmx_Val_GroupByChannel, di_buf.data(),
                              static_cast<uInt32>(di_buf.size()), &di_read,
                              &bytes_per_sample, nullptr) < 0) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "digital read failed: " + last_daqmx_error();
      break;
    }
    if (di_read != di_chunk) {
      std::lock_guard<std::mutex> lk(m_mutex);
      m_last_error = "incomplete encoder block: expected " +
                     std::to_string(di_chunk) + ", received " +
                     std::to_string(di_read);
      break;
    }

    int64_t chunk_count = 0;
    for (int32 i = 0; i < di_read; ++i) {
      const int a = di_buf[static_cast<std::size_t>(i)] ? 1 : 0;
      const int b = di_buf[static_cast<std::size_t>(di_read + i)] ? 1 : 0;
      const int state = (a << 1) | b;
      if (prev_quad_state >= 0) {
        chunk_count += kQuadratureTransitions[static_cast<std::size_t>(
            (prev_quad_state << 2) | state)];
      }
      prev_quad_state = state;
    }
    speed_chunks.emplace_back(chunk_count, di_read);
    speed_window_count += chunk_count;
    speed_window_samples += di_read;
    while (speed_chunks.size() > 1 &&
           speed_window_samples - speed_chunks.front().second >=
               window_samples) {
      speed_window_count -= speed_chunks.front().first;
      speed_window_samples -= speed_chunks.front().second;
      speed_chunks.pop_front();
    }

    const double window_time_s =
        static_cast<double>(speed_window_samples) / m_cfg.di_sample_rate_hz;
    const double raw_rpm = static_cast<double>(speed_window_count) /
                           counts_per_rev / window_time_s * 60.0;
    filtered_rpm += alpha * (raw_rpm - filtered_rpm);

    Sample s;
    s.steady_time_s = std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    analog_samples += ai_read;
    s.sample_index = analog_samples;
    s.t_s = static_cast<double>(analog_samples) / m_cfg.ai_sample_rate_hz;
    s.delivery_time_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - t0)
            .count();
    const double sign = m_cfg.invert_torque_sign ? -1.0 : 1.0;
    s.torque_nm = sign * ai_buf[static_cast<std::size_t>(ai_read - 1)] /
                  m_cfg.torque_sensitivity_v_per_nm;
    s.filtered_torque_nm =
        sign * last_filtered_v / m_cfg.torque_sensitivity_v_per_nm;
    s.speed_rpm = filtered_rpm;
    if (has_temperature) {
      double sum = 0.0;
      for (int32 i = 0; i < ai_read; ++i) {
        const double volts = ai_buf[static_cast<std::size_t>(ai_read + i)];
        if (!std::isfinite(volts)) {
          std::lock_guard<std::mutex> lk(m_mutex);
          m_last_error = "invalid AT20 voltage sample";
          m_running.store(false);
          return;
        }
        sum += volts;
      }
      const double volts = sum / ai_read;
      const double now = std::chrono::duration<double>(
          std::chrono::steady_clock::now().time_since_epoch()).count();
      s.temperature = TemperatureDaqReader::Sample{
          volts, TemperatureDaqReader::temperatureC(volts), now};
    }

    {
      std::lock_guard<std::mutex> lk(m_mutex);
      s.torque_nm -= m_tare_offset_nm;
      s.filtered_torque_nm -= m_tare_offset_nm;
      m_torque_blocks.push_back(
          {s.t_s, sign * sum_raw_v / m_cfg.torque_sensitivity_v_per_nm,
           static_cast<uint64_t>(ai_read)});
      while (m_torque_blocks.size() > 1 &&
             m_torque_blocks.front().end_s <= s.t_s - 60.0)
        m_torque_blocks.pop_front();
      m_latest = s;
    }
    m_capture.push(s);
  }
  m_running.store(false);
}

#endif // ACTUATOR_TEST_HAVE_NIDAQMX

} // namespace actuator_test
