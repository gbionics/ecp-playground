// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/external_daq.hpp"

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <vector>

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
#include <NIDAQmx.h>
#include <algorithm>
#include <cstdio>
#include <set>
#endif

namespace {
void require(bool passed, const char *message) {
  if (!passed) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
struct FakeTask {
  bool analog = false;
  bool started = false;
};
std::set<FakeTask *> tasks;
int setup_calls = 0, fail_at = 0, clears = 0, stops = 0, reads = 0;
bool warnings = false;
bool create_failure_has_handle = true;
std::atomic<bool> hold_reads{false};
std::atomic<bool> hold_after_block{false};
double ai_rate = 50000.0, di_rate = 50000.0;
enum class ReadMode { analog_error, one_block, digital_error, partial_ai, partial_di };
ReadMode read_mode = ReadMode::analog_error;

int32 setup_status() {
  return ++setup_calls == fail_at ? -1 : (warnings ? 1 : 0);
}
FakeTask *task(TaskHandle handle) {
  auto *value = static_cast<FakeTask *>(handle);
  require(tasks.count(value) == 1, "use of unowned/cleared task");
  return value;
}
void reset() {
  require(tasks.empty(), "NI task leaked");
  setup_calls = fail_at = clears = stops = reads = 0;
  warnings = false;
  create_failure_has_handle = true;
  hold_reads.store(false);
  hold_after_block.store(false);
  ai_rate = di_rate = 50000.0;
  read_mode = ReadMode::analog_error;
}
void wait_for_failure(actuator_test::ExternalDaqReader &reader) {
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (reader.running() && std::chrono::steady_clock::now() < deadline)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  require(!reader.running(), "worker did not report failure");
  require(!reader.lastError().empty(), "worker failure reason missing");
}
#endif
} // namespace

#if defined(ACTUATOR_TEST_HAVE_NIDAQMX)
int32 DAQmxGetExtendedErrorInfo(char *buffer, uInt32 size) {
  std::snprintf(buffer, size, "injected NI failure");
  return 0;
}
int32 DAQmxCreateTask(const char *, TaskHandle *handle) {
  const auto status = setup_status();
  if (status < 0 && !create_failure_has_handle)
    return status;
  // Even a failed create may return a handle that must be cleaned up.
  auto *value = new FakeTask;
  tasks.insert(value);
  *handle = value;
  return status;
}
int32 DAQmxCreateAIVoltageChan(TaskHandle handle, const char *, const char *,
                             int32, float64, float64, int32, const char *) {
  task(handle)->analog = true;
  return setup_status();
}
int32 DAQmxCreateDIChan(TaskHandle handle, const char *, const char *, int32) {
  task(handle);
  return setup_status();
}
int32 DAQmxCfgSampClkTiming(TaskHandle handle, const char *, float64, int32,
                          int32, uInt64) {
  task(handle);
  return setup_status();
}
int32 DAQmxGetSampClkRate(TaskHandle handle, float64 *rate) {
  *rate = task(handle)->analog ? ai_rate : di_rate;
  return setup_status();
}
int32 DAQmxStartTask(TaskHandle handle) {
  const auto status = setup_status();
  if (status >= 0)
    task(handle)->started = true;
  return status;
}
int32 DAQmxStopTask(TaskHandle handle) {
  require(task(handle)->started, "stopped task that was never started");
  task(handle)->started = false;
  ++stops;
  return -1; // Cleanup must still clear tasks after a stop failure.
}
int32 DAQmxClearTask(TaskHandle handle) {
  auto *value = task(handle);
  require(!value->started, "started task cleared without stopping");
  tasks.erase(value);
  delete value;
  ++clears;
  return -1;
}
int32 DAQmxReadAnalogF64(TaskHandle handle, int32 count, float64, int32,
                         float64 *buffer, uInt32 size, int32 *read, void *) {
  require(task(handle)->started && size >= static_cast<uInt32>(count),
          "invalid analog read");
  while (hold_reads.load())
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  if (read_mode == ReadMode::analog_error || reads++ > 0)
    return -1;
  if (hold_after_block.load())
    hold_reads.store(true);
  std::fill_n(buffer, count, 1.0);
  *read = read_mode == ReadMode::partial_ai ? count - 1 : count;
  return warnings ? 1 : 0;
}
int32 DAQmxReadDigitalLines(TaskHandle handle, int32 count, float64, int32,
                            uInt8 *buffer, uInt32 size, int32 *read,
                            int32 *bytes, void *) {
  require(task(handle)->started && size >= 2 * static_cast<uInt32>(count),
          "invalid digital read");
  if (read_mode == ReadMode::digital_error)
    return -1;
  std::fill_n(buffer, count * 2, 0);
  *read = read_mode == ReadMode::partial_di ? count - 1 : count;
  *bytes = 1;
  return warnings ? 1 : 0;
}
#endif

int main() {
  using actuator_test::ExternalDaqConfig;
  using actuator_test::ExternalDaqReader;
  std::string error;
#if !defined(ACTUATOR_TEST_HAVE_NIDAQMX)
  ExternalDaqReader reader;
  require(!actuator_test::external_daq_supported(), "stub reports NI support");
  require(!reader.start(error) && !error.empty(), "stub start succeeded");
  reader.stop();
  reader.stop();
  require(!reader.running() && reader.lastError().empty(), "stub state changed");
  require(!reader.tare(1.0), "stub accepted tare");
#else
  require(actuator_test::external_daq_supported(), "fake NI support missing");
  for (int failure = 1; failure <= 10; ++failure) {
    reset();
    fail_at = failure;
    ExternalDaqReader reader;
    require(!reader.start(error) && !error.empty(), "setup failure not returned");
    require(!reader.running() && tasks.empty(), "failed setup leaked tasks");
    require(clears == (failure <= 3 ? 1 : 2), "setup cleanup count incorrect");
    require(stops == (failure == 10 ? 1 : 0), "partial start cleanup incorrect");
    require(reader.lastError().find("DAQmxClearTask failed") != std::string::npos,
            "setup cleanup error not discoverable");
    if (failure == 10)
      require(reader.lastError().find("DAQmxStopTask failed") != std::string::npos,
              "partial start stop error not discoverable");
    reader.stop();
    reader.stop();
  }

  for (const int failure : {1, 4}) {
    reset();
    create_failure_has_handle = false;
    fail_at = failure;
    ExternalDaqReader reader;
    require(!reader.start(error) && tasks.empty() &&
                clears == (failure == 1 ? 0 : 1),
            "null handle creation failure cleanup incorrect");
  }

  reset();
  hold_reads.store(true);
  {
    ExternalDaqReader reader;
    require(reader.start(error) && reader.running(), "held worker start failed");
    require(reader.start(error) && setup_calls == 10,
            "start while running recreated tasks");
    hold_reads.store(false);
    wait_for_failure(reader);
    reader.stop();
    require(clears == 2 && stops == 2, "repeated start leaked tasks");
  }

  reset();
  {
    ExternalDaqReader reader;
    require(reader.start(error), "initial start failed");
    wait_for_failure(reader);
    // No stop(): start must reap the previous joinable worker itself.
    require(reader.start(error), "restart after read failure failed");
    wait_for_failure(reader);
    reader.stop();
    reader.stop();
    require(clears == 4 && stops == 4 && tasks.empty(), "restart leaked tasks");
    require(reader.lastError().find("analog read failed") != std::string::npos &&
                reader.lastError().find("DAQmxStopTask failed") != std::string::npos &&
                reader.lastError().find("DAQmxClearTask failed") != std::string::npos,
            "cleanup errors swallowed or worker reason overwritten");
  }

  for (const auto mode : {ReadMode::digital_error, ReadMode::partial_ai,
                          ReadMode::partial_di}) {
    reset();
    read_mode = mode;
    {
      ExternalDaqReader reader;
      require(reader.start(error), "read failure test setup failed");
      wait_for_failure(reader);
    }
    require(clears == 2 && stops == 2, "destructor did not release tasks");
  }

  const double bad_values[] = {0.0, -1.0,
      std::numeric_limits<double>::infinity(),
      std::numeric_limits<double>::quiet_NaN()};
  const std::vector<double ExternalDaqConfig::*> fields = {
      &ExternalDaqConfig::ai_sample_rate_hz,
      &ExternalDaqConfig::di_sample_rate_hz,
      &ExternalDaqConfig::publish_period_s,
      &ExternalDaqConfig::speed_window_s,
      &ExternalDaqConfig::encoder_ppr,
      &ExternalDaqConfig::torque_sensitivity_v_per_nm,
      &ExternalDaqConfig::lpf_cutoff_hz};
  for (const auto field : fields) {
    for (const double value : bad_values) {
      reset();
      ExternalDaqConfig cfg;
      cfg.*field = value;
      ExternalDaqReader reader(cfg);
      require(!reader.start(error) && setup_calls == 0,
              "invalid DSP configuration reached NI setup");
    }
  }
  for (const auto field : {&ExternalDaqConfig::ai_sample_rate_hz,
                          &ExternalDaqConfig::di_sample_rate_hz,
                          &ExternalDaqConfig::publish_period_s,
                          &ExternalDaqConfig::speed_window_s,
                          &ExternalDaqConfig::encoder_ppr}) {
    reset();
    ExternalDaqConfig cfg;
    cfg.*field = std::numeric_limits<double>::max();
    ExternalDaqReader reader(cfg);
    require(!reader.start(error) && setup_calls == 0,
            "overflowing conversion accepted");
  }
  reset();
  {
    ExternalDaqConfig cfg;
    cfg.lpf_cutoff_hz = cfg.ai_sample_rate_hz / 2.0;
    ExternalDaqReader reader(cfg);
    require(!reader.start(error) && setup_calls == 0, "Nyquist cutoff accepted");
  }
  reset();
  {
    ExternalDaqConfig cfg;
    cfg.di_sample_rate_hz = std::ldexp(1.0, 30);
    cfg.publish_period_s = 1.0;
    ExternalDaqReader reader(cfg);
    require(!reader.start(error) && setup_calls == 0,
            "overflowing signed DI channel indexing accepted");
  }
  for (const double rate : {0.0, 100.0,
                            std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::max()}) {
    reset();
    ai_rate = rate;
    ExternalDaqReader reader;
    require(!reader.start(error) && tasks.empty() && stops == 0,
            "invalid negotiated AI rate accepted");
  }
  reset();
  di_rate = 0;
  {
    ExternalDaqReader reader;
    require(!reader.start(error) && tasks.empty(), "invalid DI rate accepted");
  }

  reset();
  ai_rate = 200.0;
  di_rate = 400.0;
  warnings = true;
  read_mode = ReadMode::one_block;
  {
    ExternalDaqConfig cfg;
    cfg.ai_sample_rate_hz = cfg.di_sample_rate_hz = 1000.0;
    cfg.publish_period_s = 0.005;
    cfg.lpf_cutoff_hz = 20.0;
    ExternalDaqReader reader(cfg);
    reader.captureBuffer().start();
    require(reader.start(error), "NI warnings treated as setup errors");
    wait_for_failure(reader);
    reader.stop();
    const auto sample = reader.latest();
    const double wc = std::tan(std::acos(-1.0) * cfg.lpf_cutoff_hz / ai_rate);
    const double expected = wc * wc / (1 + std::sqrt(2.0) * wc + wc * wc) /
                            cfg.torque_sensitivity_v_per_nm;
    require(std::abs(sample.filtered_torque_nm - expected) < 1e-10,
            "filter did not use negotiated AI rate");
    require(sample.sample_index == 1 && sample.t_s == 0.005 &&
                sample.delivery_time_s >= 0.0 && sample.torque_nm == 40.0,
            "acquisition timestamps/raw torque changed");
    require(reader.analogSampleRateHz() == ai_rate &&
                reader.publicationPeriodS() == 0.005,
            "negotiated rate not exposed");
    std::vector<ExternalDaqReader::Sample> captured;
    require(reader.captureBuffer().drain(captured, true) == 0 &&
                captured.size() == 1 && captured[0].sample_index == 1,
            "capture buffering changed");
  }
  require(tasks.empty() && clears == 2 && stops == 2, "successful run leaked");

  reset();
  read_mode = ReadMode::one_block;
  hold_after_block.store(true);
  {
    ExternalDaqReader reader;
    reader.captureBuffer().start();
    require(reader.start(error), "tare/restart setup failed");
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (reader.latest().sample_index == 0 &&
           std::chrono::steady_clock::now() < deadline)
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    require(reader.latest().sample_index > 0 && reader.tare(5.0),
            "initial sample/tare unavailable");
    hold_reads.store(false);
    wait_for_failure(reader);
    const auto previous_index = reader.latest().sample_index;
    hold_reads.store(true);
    hold_after_block.store(false);
    read_mode = ReadMode::analog_error;
    require(reader.start(error) && reader.running(), "held restart failed");
    const auto latest = reader.latest();
    require(latest.sample_index == 0 && latest.t_s == 0.0 &&
                latest.delivery_time_s == 0.0 && latest.torque_nm == 0.0 &&
                latest.filtered_torque_nm == 0.0 && latest.speed_rpm == 0.0,
            "restart exposed stale latest sample");
    require(reader.tareOffsetNm() == 5.0 && reader.lastError().empty(),
            "restart lost cumulative tare or retained previous error");
    hold_reads.store(false);
    wait_for_failure(reader);
    reader.stop();
    std::vector<ExternalDaqReader::Sample> captured;
    require(reader.captureBuffer().drain(captured, true) == 0 &&
                captured.size() == 1 &&
                captured.front().sample_index == previous_index,
            "restart changed capture activation/content");
  }
#endif
}
