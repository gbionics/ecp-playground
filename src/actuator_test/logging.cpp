#include "actuator_test/logging.hpp"

#include "actuator_test/math.hpp"
#include "actuator_test/session.hpp"
#include "actuator_test/settings.hpp"

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>

namespace actuator_test {

namespace {

constexpr std::size_t k_log_batch_samples = 512;

bool mkdir_if_needed(const std::string &path) {
  if (path.empty()) {
    return false;
  }
  if (::mkdir(path.c_str(), 0775) == 0 || errno == EEXIST) {
    return true;
  }
  return false;
}

bool mkdirs_recursive(const std::string &path) {
  if (path.empty()) {
    return false;
  }

  std::string partial;
  partial.reserve(path.size());
  for (char ch : path) {
    partial.push_back(ch);
    if (ch == '/') {
      if (partial.size() > 1 &&
          !mkdir_if_needed(partial.substr(0, partial.size() - 1))) {
        return false;
      }
    }
  }
  return mkdir_if_needed(path);
}

} // namespace

JointCsvLogger::~JointCsvLogger() { close(); }

bool JointCsvLogger::open(const std::string &path,
                          const RuntimeProfile &profile, const JointHandle &jh,
                          const JointPlan &plan, const Trajectory &traj) {
  m_path = path;
  m_file = std::fopen(path.c_str(), "w");
  if (m_file == nullptr) {
    return false;
  }

  m_buffer.resize(profile.log_file_buffer_bytes);
  std::setvbuf(m_file, m_buffer.data(), _IOFBF, m_buffer.size());
  m_encoder_bits = jh.driver->encoder_bits();
  m_active_samples.clear();
  m_pending_samples.clear();
  m_active_samples.reserve(k_log_batch_samples);
  m_pending_samples.reserve(k_log_batch_samples);
  m_stop_requested = false;
  m_pending_ready = false;

  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  ::localtime_r(&now, &tm_buf);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tm_buf);

  std::fprintf(m_file, "# actuator-test log\n");
  std::fprintf(m_file, "# timestamp, %s\n", stamp);
  std::fprintf(m_file, "# joint_name, %s\n", jh.name.c_str());
  std::fprintf(m_file, "# driver, %s\n", jh.driver_name.c_str());
  std::fprintf(m_file, "# alias, %u\n", static_cast<unsigned>(jh.alias));
  std::fprintf(m_file, "# model, %s\n",
               jh.model.empty() ? "(default)" : jh.model.c_str());
  std::fprintf(m_file, "# operation_mode, %s\n",
               jh.operation_mode_name.c_str());
  std::fprintf(m_file, "# encoder_bits, %u\n",
               static_cast<unsigned>(jh.driver->encoder_bits()));
  std::fprintf(m_file, "# pvt_kp, %d\n", jh.pvt_kp);
  std::fprintf(m_file, "# pvt_kd, %d\n", jh.pvt_kd);
  std::fprintf(m_file, "# min_counts, %d\n", plan.min_counts);
  std::fprintf(m_file, "# max_counts, %d\n", plan.max_counts);
  std::fprintf(m_file, "# start_counts, %d\n", plan.start_counts);
  std::fprintf(m_file, "# min_deg, %.10f\n",
               counts2deg(plan.min_counts, jh.driver->encoder_bits()));
  std::fprintf(m_file, "# max_deg, %.10f\n",
               counts2deg(plan.max_counts, jh.driver->encoder_bits()));
  std::fprintf(m_file, "# start_deg, %.10f\n",
               counts2deg(plan.start_counts, jh.driver->encoder_bits()));
  {
    std::ostringstream header;
    traj.describe_csv(header);
    const std::string header_text = header.str();
    std::fwrite(header_text.data(), 1, header_text.size(), m_file);
  }
  std::fprintf(m_file, "# loop_rate_hz, %.10f\n", profile.loop_rate_hz);
  std::fprintf(m_file, "# lpf_cutoff_hz, %.10f\n", profile.lpf_cutoff_hz);
  std::fprintf(m_file, "# approach_seconds, %.10f\n", plan.approach_T);
  std::fprintf(m_file, "# pre_ramp_hold_seconds, %.10f\n",
               profile.pre_ramp_hold_seconds);
  std::fprintf(m_file, "# max_approach_speed_deg_s, %.10f\n",
               profile.max_approach_speed_deg_s);
  std::fprintf(m_file, "# temp_warn_celsius, %d\n", profile.temp_warn_celsius);
  std::fprintf(m_file, "# temp_abort_celsius, %d\n",
               profile.temp_abort_celsius);
  std::fprintf(
      m_file,
      "t_s,phase,phase_t_s,ref_raw_counts,ref_filt_counts,actual_counts,"
      "ref_raw_deg,ref_filt_deg,actual_deg,motor_temp_c,drive_temp_c,error_"
      "code\n");

  m_writer_thread = std::thread(&JointCsvLogger::writer_main, this);

  return true;
}

bool JointCsvLogger::open_plain(const std::string &path,
                                const RuntimeProfile &profile,
                                const JointHandle &jh, int32_t min_counts,
                                int32_t max_counts, int32_t start_counts) {
  m_path = path;
  m_file = std::fopen(path.c_str(), "w");
  if (m_file == nullptr) {
    return false;
  }

  m_buffer.resize(profile.log_file_buffer_bytes);
  std::setvbuf(m_file, m_buffer.data(), _IOFBF, m_buffer.size());
  m_encoder_bits = jh.driver->encoder_bits();
  m_active_samples.clear();
  m_pending_samples.clear();
  m_active_samples.reserve(k_log_batch_samples);
  m_pending_samples.reserve(k_log_batch_samples);
  m_stop_requested = false;
  m_pending_ready = false;

  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  ::localtime_r(&now, &tm_buf);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%S", &tm_buf);

  std::fprintf(m_file, "# actuator-test log\n");
  std::fprintf(m_file, "# kind, free-recording\n");
  std::fprintf(m_file, "# timestamp, %s\n", stamp);
  std::fprintf(m_file, "# joint_name, %s\n", jh.name.c_str());
  std::fprintf(m_file, "# driver, %s\n", jh.driver_name.c_str());
  std::fprintf(m_file, "# alias, %u\n", static_cast<unsigned>(jh.alias));
  std::fprintf(m_file, "# model, %s\n",
               jh.model.empty() ? "(default)" : jh.model.c_str());
  std::fprintf(m_file, "# operation_mode, %s\n",
               jh.operation_mode_name.c_str());
  std::fprintf(m_file, "# encoder_bits, %u\n",
               static_cast<unsigned>(jh.driver->encoder_bits()));
  std::fprintf(m_file, "# pvt_kp, %d\n", jh.pvt_kp);
  std::fprintf(m_file, "# pvt_kd, %d\n", jh.pvt_kd);
  std::fprintf(m_file, "# min_counts, %d\n", min_counts);
  std::fprintf(m_file, "# max_counts, %d\n", max_counts);
  std::fprintf(m_file, "# start_counts, %d\n", start_counts);
  std::fprintf(m_file, "# min_deg, %.10f\n",
               counts2deg(min_counts, jh.driver->encoder_bits()));
  std::fprintf(m_file, "# max_deg, %.10f\n",
               counts2deg(max_counts, jh.driver->encoder_bits()));
  std::fprintf(m_file, "# start_deg, %.10f\n",
               counts2deg(start_counts, jh.driver->encoder_bits()));
  std::fprintf(m_file, "# loop_rate_hz, %.10f\n", profile.loop_rate_hz);
  std::fprintf(m_file, "# lpf_cutoff_hz, %.10f\n", profile.lpf_cutoff_hz);
  std::fprintf(m_file, "# temp_warn_celsius, %d\n", profile.temp_warn_celsius);
  std::fprintf(m_file, "# temp_abort_celsius, %d\n",
               profile.temp_abort_celsius);
  std::fprintf(
      m_file,
      "t_s,phase,phase_t_s,ref_raw_counts,ref_filt_counts,actual_counts,"
      "ref_raw_deg,ref_filt_deg,actual_deg,motor_temp_c,drive_temp_c,error_"
      "code\n");

  m_writer_thread = std::thread(&JointCsvLogger::writer_main, this);

  return true;
}

void JointCsvLogger::write(const LogSample &sample) {
  if (m_file == nullptr) {
    return;
  }

  m_active_samples.push_back(sample);
  if (m_active_samples.size() >= k_log_batch_samples) {
    flush_active_batch();
  }
}

void JointCsvLogger::close() {
  if (m_file == nullptr) {
    return;
  }

  flush_active_batch();
  {
    std::unique_lock<std::mutex> lock(m_mutex);
    m_cv.wait(lock, [this] { return !m_pending_ready; });
    m_stop_requested = true;
  }
  m_cv.notify_one();

  if (m_writer_thread.joinable()) {
    m_writer_thread.join();
  }

  std::fflush(m_file);
  std::fclose(m_file);
  m_file = nullptr;
}

const std::string &JointCsvLogger::path() const noexcept { return m_path; }

bool JointCsvLogger::is_open() const noexcept { return m_file != nullptr; }

void JointCsvLogger::flush_active_batch() {
  if (m_active_samples.empty()) {
    return;
  }

  std::unique_lock<std::mutex> lock(m_mutex);
  m_cv.wait(lock, [this] { return !m_pending_ready; });
  m_pending_samples.swap(m_active_samples);
  m_pending_ready = true;
  lock.unlock();
  m_cv.notify_one();
}

void JointCsvLogger::writer_main() {
  for (;;) {
    std::vector<LogSample> batch;
    {
      std::unique_lock<std::mutex> lock(m_mutex);
      m_cv.wait(lock, [this] { return m_pending_ready || m_stop_requested; });
      if (!m_pending_ready && m_stop_requested) {
        break;
      }
      batch.swap(m_pending_samples);
      m_pending_ready = false;
    }
    m_cv.notify_one();
    write_batch(batch);
    batch.clear();
  }
}

void JointCsvLogger::write_batch(const std::vector<LogSample> &batch) {
  if (m_file == nullptr) {
    return;
  }

  char line[256];
  for (const LogSample &s : batch) {
    const int written = std::snprintf(
        line, sizeof(line),
        "%.9f,%d,%.9f,%.6f,%.6f,%d,%.9f,%.9f,%.9f,%d,%d,%u\n", s.t_s, s.phase,
        s.phase_t_s, s.ref_raw_counts, s.ref_filt_counts, s.actual_counts,
        counts2deg(static_cast<int32_t>(s.ref_raw_counts), m_encoder_bits),
        counts2deg(static_cast<int32_t>(s.ref_filt_counts), m_encoder_bits),
        counts2deg(s.actual_counts, m_encoder_bits),
        static_cast<int>(s.motor_temp_c), static_cast<int>(s.drive_temp_c),
        static_cast<unsigned>(s.error_code));
    if (written > 0) {
      std::fwrite(line, 1, static_cast<std::size_t>(written), m_file);
    }
  }
}

std::string make_run_log_dir(const RuntimeProfile &profile) {
  const char *env_root = std::getenv("ACTUATOR_TEST_LOG_DIR");
  const std::string root_dir = (env_root != nullptr && env_root[0] != '\0')
                                   ? std::string(env_root)
                                   : profile.log_root_dir;
  if (!mkdirs_recursive(root_dir)) {
    std::fprintf(
        stderr,
        "warning: cannot create log root '%s' (errno=%d) -- logging disabled\n",
        root_dir.c_str(), errno);
    return {};
  }

  const std::time_t now = std::time(nullptr);
  std::tm tm_buf{};
  ::localtime_r(&now, &tm_buf);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y%m%d-%H%M%S", &tm_buf);

  std::string dir = root_dir + "/" + stamp;
  if (!mkdir_if_needed(dir)) {
    std::fprintf(
        stderr,
        "warning: cannot create log dir '%s' (errno=%d) -- logging disabled\n",
        dir.c_str(), errno);
    return {};
  }

  return dir;
}

} // namespace actuator_test
