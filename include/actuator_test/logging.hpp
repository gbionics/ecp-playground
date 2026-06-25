#pragma once

#include "actuator_test/device.hpp"
#include "actuator_test/settings.hpp"
#include "actuator_test/trajectory.hpp"

#include <cstdint>
#include <condition_variable>
#include <cstdio>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace actuator_test
{

struct JointPlan;

struct LogSample
{
    double t_s = 0.0;
    int phase = 0;
    double phase_t_s = 0.0;
    double ref_raw_counts = 0.0;
    double ref_filt_counts = 0.0;
    int32_t actual_counts = 0;
    int16_t motor_temp_c = -1;
    int16_t drive_temp_c = -1;
    uint16_t error_code = 0;
};

class JointCsvLogger
{
public:
    JointCsvLogger() = default;
    ~JointCsvLogger();

    bool open(const std::string &path, const RuntimeProfile &profile, const JointHandle &jh, const JointPlan &plan,
              const Trajectory &traj);
    void write(const LogSample &sample);
    void close();

    const std::string &path() const noexcept;
    bool is_open() const noexcept;

    JointCsvLogger(const JointCsvLogger &) = delete;
    JointCsvLogger &operator=(const JointCsvLogger &) = delete;
    JointCsvLogger(JointCsvLogger &&) = delete;
    JointCsvLogger &operator=(JointCsvLogger &&) = delete;

private:
    void flush_active_batch();
    void writer_main();
    void write_batch(const std::vector<LogSample> &batch);

    FILE *m_file = nullptr;
    std::string m_path;
    std::vector<char> m_buffer;
    std::vector<LogSample> m_active_samples;
    std::vector<LogSample> m_pending_samples;
    std::thread m_writer_thread;
    std::mutex m_mutex;
    std::condition_variable m_cv;
    bool m_stop_requested = false;
    bool m_pending_ready = false;
    int m_encoder_bits = 0;
};

std::string make_run_log_dir(const RuntimeProfile &profile);

} // namespace actuator_test
