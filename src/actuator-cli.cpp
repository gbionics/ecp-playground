// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

// -----------------------------------------------------------------------------
//  Standalone build.
//
//  This file is a self-contained version of the actuator-test console tool.
//  All declarations and implementations that normally live in the
//  actuator_test/*.hpp headers and src/actuator_test/*.cpp translation units
//  have been inlined directly below, so the only dependencies are
//  <ethercat-primer/core> and the C++ standard / POSIX libraries.
// -----------------------------------------------------------------------------

#include <ethercat-primer/core>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <ostream>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

#include <sched.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

// =============================================================================
//  Declarations (inlined from include/actuator_test/*.hpp)
// =============================================================================

// ---- types.hpp --------------------------------------------------------------
namespace actuator_test {

enum class TrajectoryMode {
  Sin,         ///< Fixed-frequency sinusoid about the captured mid-point.
  ChirpLinear, ///< Linear-frequency chirp (sweep) f0 -> f1 -> f0.
  ChirpLog,    ///< Logarithmic (exponential) frequency chirp.
  Triangle,    ///< Constant-velocity triangle position sweep.
  Step,        ///< Square wave (step response characterisation).
  Multisine,   ///< Sum of harmonics with Schroeder phasing (frequency ID).
  Spline,      ///< Replay a Catmull-Rom spline recorded by backdriving.
};

inline const char *mode_to_string(TrajectoryMode mode) noexcept {
  switch (mode) {
  case TrajectoryMode::Sin:
    return "sin";
  case TrajectoryMode::ChirpLinear:
    return "chirp-linear";
  case TrajectoryMode::ChirpLog:
    return "chirp-log";
  case TrajectoryMode::Triangle:
    return "triangle";
  case TrajectoryMode::Step:
    return "step";
  case TrajectoryMode::Multisine:
    return "multisine";
  case TrajectoryMode::Spline:
    return "spline";
  }
  return "?";
}

/// True when the mode is "trained" by recording a waypoint trail during the
/// idle capture phase (currently only spline).  Parametric modes only need
/// the captured min/max envelope.
inline bool mode_requires_recording(TrajectoryMode mode) noexcept {
  return mode == TrajectoryMode::Spline;
}

/// True when the trajectory is generated analytically from the captured
/// [min, max] envelope (everything except spline).
inline bool mode_is_parametric(TrajectoryMode mode) noexcept {
  return mode != TrajectoryMode::Spline;
}

} // namespace actuator_test

// ---- settings.hpp -----------------------------------------------------------
namespace actuator_test {

struct SettingsDefaults {
  // Control and logging run on the same deterministic cadence.
  static constexpr double k_loop_rate_hz = 1000.0;
  static constexpr double k_lpf_cutoff_hz = 10.0;

  static constexpr double k_traj_freq_hz = 0.5;
  static constexpr double k_traj_safety_factor = 0.8;

  // Frequency-sweep / chirp characterisation.
  static constexpr double k_chirp_f0_hz = 0.1;
  static constexpr double k_chirp_f1_hz = 5.0;
  static constexpr double k_chirp_sweep_seconds = 20.0;
  // Triangle / step sweep cadence.
  static constexpr double k_triangle_cycle_seconds = 4.0;
  static constexpr double k_step_cycle_seconds = 4.0;
  // Multisine identification.
  static constexpr double k_multisine_base_hz = 0.1;
  static constexpr int k_multisine_harmonics = 10;

  static constexpr double k_approach_seconds = 2.0;
  static constexpr double k_max_approach_speed_deg_s = 100.0;
  static constexpr double k_pre_ramp_hold_seconds = 0.5;

  static constexpr double k_status_print_interval_s = 1.0;
  static constexpr double k_min_range_deg = 2.0;

  static constexpr double k_waypoint_record_rate_hz = 20.0;
  static constexpr double k_waypoint_record_period_s =
      1.0 / k_waypoint_record_rate_hz;
  static constexpr double k_waypoint_decimation_deg = 0.1;
  static constexpr std::size_t k_min_spline_waypoints = 3;
  static constexpr std::size_t k_max_spline_waypoints = 256;
  static constexpr double k_min_segment_seconds = 0.1;

  static constexpr int16_t k_temp_warn_celsius = 60;
  static constexpr int16_t k_temp_abort_celsius = 80;

  static constexpr const char *k_log_root_dir = "logs/actuator-test";
  static constexpr std::size_t k_log_file_buffer_bytes = 1U << 20;
  static constexpr bool k_enable_realtime_scheduler = true;
  static constexpr int k_realtime_priority = 10;
};

struct RuntimeProfile {
  double loop_rate_hz = SettingsDefaults::k_loop_rate_hz;
  double lpf_cutoff_hz = SettingsDefaults::k_lpf_cutoff_hz;
  double traj_freq_hz = SettingsDefaults::k_traj_freq_hz;
  double traj_safety_factor = SettingsDefaults::k_traj_safety_factor;
  double chirp_f0_hz = SettingsDefaults::k_chirp_f0_hz;
  double chirp_f1_hz = SettingsDefaults::k_chirp_f1_hz;
  double chirp_sweep_seconds = SettingsDefaults::k_chirp_sweep_seconds;
  double triangle_cycle_seconds = SettingsDefaults::k_triangle_cycle_seconds;
  double step_cycle_seconds = SettingsDefaults::k_step_cycle_seconds;
  double multisine_base_hz = SettingsDefaults::k_multisine_base_hz;
  int multisine_harmonics = SettingsDefaults::k_multisine_harmonics;
  double approach_seconds = SettingsDefaults::k_approach_seconds;
  double max_approach_speed_deg_s =
      SettingsDefaults::k_max_approach_speed_deg_s;
  double pre_ramp_hold_seconds = SettingsDefaults::k_pre_ramp_hold_seconds;
  double status_print_interval_s = SettingsDefaults::k_status_print_interval_s;
  double min_range_deg = SettingsDefaults::k_min_range_deg;
  double waypoint_record_rate_hz = SettingsDefaults::k_waypoint_record_rate_hz;
  double waypoint_decimation_deg = SettingsDefaults::k_waypoint_decimation_deg;
  std::size_t min_spline_waypoints = SettingsDefaults::k_min_spline_waypoints;
  std::size_t max_spline_waypoints = SettingsDefaults::k_max_spline_waypoints;
  double min_segment_seconds = SettingsDefaults::k_min_segment_seconds;
  int16_t temp_warn_celsius = SettingsDefaults::k_temp_warn_celsius;
  int16_t temp_abort_celsius = SettingsDefaults::k_temp_abort_celsius;
  std::string log_root_dir = SettingsDefaults::k_log_root_dir;
  std::size_t log_file_buffer_bytes = SettingsDefaults::k_log_file_buffer_bytes;
  bool enable_realtime_scheduler =
      SettingsDefaults::k_enable_realtime_scheduler;
  int realtime_priority = SettingsDefaults::k_realtime_priority;

  double loop_period_s() const noexcept { return 1.0 / loop_rate_hz; }

  double waypoint_record_period_s() const noexcept {
    return 1.0 / waypoint_record_rate_hz;
  }
};

RuntimeProfile load_runtime_profile(const ecp::DeviceConfig &cfg);

} // namespace actuator_test

// ---- math.hpp ---------------------------------------------------------------
namespace actuator_test {

inline constexpr double k_pi = 3.14159265358979323846;

inline double counts2deg(int32_t counts, int encoder_bits) noexcept {
  const int64_t cpr = int64_t{1} << encoder_bits;
  return static_cast<double>(counts) / static_cast<double>(cpr) * 360.0;
}

inline int32_t deg2counts(double deg, int encoder_bits) noexcept {
  const int64_t cpr = int64_t{1} << encoder_bits;
  return static_cast<int32_t>(
      std::llround(deg / 360.0 * static_cast<double>(cpr)));
}

class LowPass {
public:
  LowPass(double cutoff_hz, double dt_s, double initial) noexcept
      : m_alpha(1.0 - std::exp(-2.0 * k_pi * cutoff_hz * dt_s)), m_y(initial) {}

  double step(double x) noexcept {
    m_y += m_alpha * (x - m_y);
    return m_y;
  }

private:
  double m_alpha;
  double m_y;
};

inline double min_jerk(double t, double T, double p0, double p1) noexcept {
  if (T <= 0.0 || t >= T) {
    return p1;
  }
  if (t <= 0.0) {
    return p0;
  }
  const double tau = t / T;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double tau4 = tau3 * tau;
  const double tau5 = tau4 * tau;
  const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
  return p0 + (p1 - p0) * s;
}

} // namespace actuator_test

// ---- device.hpp -------------------------------------------------------------
namespace actuator_test {

enum class DriverKind {
  MyActuator,
  Novanta,
};

class DriverAdapter {
public:
  virtual ~DriverAdapter() = default;

  virtual DriverKind kind() const noexcept = 0;
  virtual const char *kind_name() const noexcept = 0;
  virtual int32_t actual_position() const noexcept = 0;
  virtual int32_t actual_velocity() const noexcept = 0;
  virtual uint16_t status() const noexcept = 0;
  virtual bool fault() const noexcept = 0;
  virtual uint16_t error_code() const noexcept = 0;

  virtual bool has_temperature_feedback() const noexcept = 0;
  virtual int16_t motor_temperature() const noexcept = 0;
  virtual int16_t drive_temperature() const noexcept = 0;
  virtual int8_t operation_mode_display() const noexcept = 0;

  virtual void set_target_position(int32_t value) noexcept = 0;
  virtual void set_target_velocity(int32_t value) noexcept = 0;
  virtual void update_operation_mode(int8_t op_mode) noexcept = 0;
  virtual void idle() noexcept = 0;

  virtual uint16_t encoder_bits() const noexcept { return 0; }

  virtual void apply_runtime_gains(int32_t, int32_t) noexcept {}
};

struct JointHandle {
  std::string name;
  uint16_t alias = 0;
  std::string driver_name;
  std::string model;
  std::string operation_mode_name;
  int8_t operation_mode_code = ecp::DS402::OP_HOM;
  int encoder_bits = 17;
  int32_t pvt_kp = 200;
  int32_t pvt_kd = 50;
  bool selectable = false;
  std::string unavailable_reason;
  std::shared_ptr<DriverAdapter> driver;
};

class DriverFactory {
public:
  virtual ~DriverFactory() = default;

  virtual std::string_view driver_name() const noexcept = 0;
  virtual std::optional<JointHandle>
  create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
         const std::string &device_name) const = 0;
};

std::optional<int8_t> ds402_mode_from_name(const std::string &name);

const DriverFactory *find_driver_factory(const std::string &driver_name);

std::string supported_driver_names();

std::vector<JointHandle> enumerate_joints(const ecp::EthercatBus &bus,
                                          const ecp::DeviceConfig &cfg);

std::vector<std::size_t> pick_joints(const std::vector<JointHandle> &joints);

std::optional<TrajectoryMode> pick_mode(TrajectoryMode previous);

void idle_all(std::vector<JointHandle> &joints);

} // namespace actuator_test

// ---- safety.hpp -------------------------------------------------------------
namespace actuator_test {

struct SafetyState {
  uint16_t last_error = 0;
};

bool safety_violated(const DriverAdapter &drv, const RuntimeProfile &profile,
                     const std::string &joint_name, SafetyState &state,
                     std::string &reason);

} // namespace actuator_test

// ---- trajectory.hpp ---------------------------------------------------------
namespace actuator_test {

class Trajectory {
public:
  virtual ~Trajectory() = default;
  virtual double approach_target() const noexcept = 0;
  virtual double sample(double t_s) const noexcept = 0;
  virtual void describe_csv(std::ostream &os) const = 0;
};

class SinTrajectory final : public Trajectory {
public:
  SinTrajectory(double centre_counts, double amp_counts, double freq_hz,
                double safety_factor, int encoder_bits) noexcept;

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

private:
  double m_centre;
  double m_amp;
  double m_freq;
  double m_safety_factor;
  int m_enc_bits;
};

/// Frequency-sweep (chirp) about a centre with mirrored-time playback so the
/// frequency ramps f0 -> f1 -> f0 continuously and indefinitely.  Supports
/// both linear and logarithmic (exponential) frequency progression.  The
/// instantaneous frequency profile is a continuous triangle wave, so the
/// generated position is C1-continuous (no velocity step at turnarounds).
class ChirpTrajectory final : public Trajectory {
public:
  ChirpTrajectory(double centre_counts, double amp_counts, double f0_hz,
                  double f1_hz, double sweep_seconds, bool logarithmic,
                  int encoder_bits) noexcept;

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

private:
  double phase_at(double t_s) const noexcept;
  double sweep_integral(double x)
      const noexcept; ///< F(x) = integral of f over [0, x] of the up-sweep.

  double m_centre;
  double m_amp;
  double m_f0;
  double m_f1;
  double m_sweep;
  bool m_log;
  int m_enc_bits;
};

/// Constant-velocity triangle sweep between (centre - amp) and (centre + amp).
/// Starts at the centre rising, useful for slow full-range mechanical sweeps.
class TriangleTrajectory final : public Trajectory {
public:
  TriangleTrajectory(double centre_counts, double amp_counts,
                     double cycle_seconds, int encoder_bits) noexcept;

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

private:
  double m_centre;
  double m_amp;
  double m_cycle;
  int m_enc_bits;
};

/// Square-wave step trajectory alternating between centre +/- amp every half
/// cycle.  The reference low-pass filter shapes the commanded edge, so this
/// yields a repeatable step-response characterisation input.
class StepTrajectory final : public Trajectory {
public:
  StepTrajectory(double centre_counts, double amp_counts, double cycle_seconds,
                 int encoder_bits) noexcept;

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

private:
  double m_centre;
  double m_amp;
  double m_cycle;
  int m_enc_bits;
};

/// Sum-of-sines (multisine) trajectory with Schroeder phasing to minimise the
/// crest factor.  Excites several frequencies simultaneously, which is ideal
/// for one-shot frequency-response identification.
class MultisineTrajectory final : public Trajectory {
public:
  MultisineTrajectory(double centre_counts, double amp_counts,
                      double base_freq_hz, int harmonics, int encoder_bits);

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

private:
  double m_centre;
  double m_amp;
  double m_base_freq;
  int m_harmonics;
  int m_enc_bits;
  std::vector<double> m_phases;
};

/// Build an analytic (non-recorded) trajectory for one joint from its captured
/// [centre, amplitude] envelope.  Shared by the console session builder and the
/// GUI controller so both map each mode to a generator identically.  Spline is
/// recorded, not parametric, so passing TrajectoryMode::Spline falls back to a
/// sinusoid (callers build splines explicitly from recorded waypoints).
std::unique_ptr<Trajectory>
make_parametric_trajectory(TrajectoryMode mode, double centre, double amp,
                           const RuntimeProfile &profile, int encoder_bits);

class SplineTrajectory final : public Trajectory {
public:
  SplineTrajectory(std::vector<double> knot_times,
                   std::vector<int32_t> waypoint_counts, int encoder_bits);

  double approach_target() const noexcept override;
  double sample(double t_s) const noexcept override;
  void describe_csv(std::ostream &os) const override;

  double total_one_way_seconds() const noexcept;
  std::size_t waypoint_count() const noexcept;

private:
  std::vector<double> m_t;
  std::vector<double> m_w;
  std::vector<double> m_m;
  double m_T = 0.0;
  int m_enc_bits;
};

} // namespace actuator_test

// ---- logging.hpp ------------------------------------------------------------
namespace actuator_test {

struct JointPlan;

struct LogSample {
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

class JointCsvLogger {
public:
  JointCsvLogger() = default;
  ~JointCsvLogger();

  bool open(const std::string &path, const RuntimeProfile &profile,
            const JointHandle &jh, const JointPlan &plan,
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

// ---- session.hpp ------------------------------------------------------------
namespace actuator_test {

enum class ControlPhase {
  Hold = 0,
  Approach = 1,
  Trajectory = 2,
};

struct JointPlan {
  JointHandle *jh = nullptr;
  int32_t min_counts = 0;
  int32_t max_counts = 0;
  std::vector<int32_t> waypoint_counts;
  std::unique_ptr<Trajectory> traj;

  int32_t start_counts = 0;
  double approach_T = 0.0;
  ControlPhase phase = ControlPhase::Hold;
  double phase_t = 0.0;
  std::unique_ptr<LowPass> lpf;
  SafetyState safety;

  std::unique_ptr<JointCsvLogger> csv_logger;
};

bool capture_limits_multi(std::vector<JointPlan> &plans, TrajectoryMode mode,
                          const RuntimeProfile &profile);

void run_trajectory_multi(std::vector<JointPlan> &plans, TrajectoryMode mode,
                          const RuntimeProfile &profile);

} // namespace actuator_test

// ---- terminal.hpp -----------------------------------------------------------
namespace actuator_test {

class RawTty {
public:
  RawTty() noexcept : m_active(false) {
    if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &m_saved) != 0) {
      return;
    }
    termios raw = m_saved;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0) {
      m_active = true;
    }
  }

  ~RawTty() {
    if (m_active) {
      tcsetattr(STDIN_FILENO, TCSANOW, &m_saved);
    }
  }

  RawTty(const RawTty &) = delete;
  RawTty &operator=(const RawTty &) = delete;

  int try_read() const noexcept {
    unsigned char c = 0;
    const ssize_t n = ::read(STDIN_FILENO, &c, 1);
    return (n == 1) ? static_cast<int>(c) : -1;
  }

private:
  termios m_saved{};
  bool m_active;
};

} // namespace actuator_test

// ---- runtime.hpp ------------------------------------------------------------
namespace actuator_test {

struct CapabilityStatus {
  bool net_admin = false;
  bool net_raw = false;
  bool sys_nice = false;

  bool all_satisfied(bool require_sys_nice = true) const noexcept {
    return net_admin && net_raw && (!require_sys_nice || sys_nice);
  }
};

void register_signal_handlers();

CapabilityStatus query_process_capabilities();

std::string capability_fix_command(const std::string &executable_path);

bool ensure_runtime_capabilities(const std::string &executable_path,
                                 const RuntimeProfile &profile);

void try_enable_realtime_scheduler(const RuntimeProfile &profile);

} // namespace actuator_test

// =============================================================================
//  Implementations (inlined from src/actuator_test/*.cpp)
// =============================================================================

// ---- settings.cpp -----------------------------------------------------------
namespace actuator_test {

namespace {

template <typename T>
T config_value(const ecp::DeviceConfig &cfg, const char *key,
               const T &fallback) {
  return cfg.get<T>("actuator_test", key, fallback);
}

} // namespace

RuntimeProfile load_runtime_profile(const ecp::DeviceConfig &cfg) {
  RuntimeProfile profile;

  profile.loop_rate_hz = std::max(
      1.0, config_value<double>(cfg, "loop_rate_hz", profile.loop_rate_hz));
  profile.lpf_cutoff_hz = std::max(
      0.01, config_value<double>(cfg, "lpf_cutoff_hz", profile.lpf_cutoff_hz));
  profile.traj_freq_hz = std::max(
      0.001, config_value<double>(cfg, "traj_freq_hz", profile.traj_freq_hz));
  profile.traj_safety_factor =
      std::clamp(config_value<double>(cfg, "traj_safety_factor",
                                      profile.traj_safety_factor),
                 0.0, 1.0);
  profile.chirp_f0_hz = std::max(
      1e-4, config_value<double>(cfg, "chirp_f0_hz", profile.chirp_f0_hz));
  profile.chirp_f1_hz = std::max(
      1e-4, config_value<double>(cfg, "chirp_f1_hz", profile.chirp_f1_hz));
  profile.chirp_sweep_seconds =
      std::max(0.1, config_value<double>(cfg, "chirp_sweep_seconds",
                                         profile.chirp_sweep_seconds));
  profile.triangle_cycle_seconds =
      std::max(0.1, config_value<double>(cfg, "triangle_cycle_seconds",
                                         profile.triangle_cycle_seconds));
  profile.step_cycle_seconds =
      std::max(0.1, config_value<double>(cfg, "step_cycle_seconds",
                                         profile.step_cycle_seconds));
  profile.multisine_base_hz =
      std::max(1e-4, config_value<double>(cfg, "multisine_base_hz",
                                          profile.multisine_base_hz));
  profile.multisine_harmonics =
      std::clamp(config_value<int>(cfg, "multisine_harmonics",
                                   profile.multisine_harmonics),
                 1, 64);
  profile.approach_seconds =
      std::max(0.0, config_value<double>(cfg, "approach_seconds",
                                         profile.approach_seconds));
  profile.max_approach_speed_deg_s =
      std::max(0.1, config_value<double>(cfg, "max_approach_speed_deg_s",
                                         profile.max_approach_speed_deg_s));
  profile.pre_ramp_hold_seconds =
      std::max(0.0, config_value<double>(cfg, "pre_ramp_hold_seconds",
                                         profile.pre_ramp_hold_seconds));
  profile.status_print_interval_s =
      std::max(0.1, config_value<double>(cfg, "status_print_interval_s",
                                         profile.status_print_interval_s));
  profile.min_range_deg = std::max(
      0.0, config_value<double>(cfg, "min_range_deg", profile.min_range_deg));
  profile.waypoint_record_rate_hz =
      std::max(1.0, config_value<double>(cfg, "waypoint_record_rate_hz",
                                         profile.waypoint_record_rate_hz));
  profile.waypoint_decimation_deg =
      std::max(0.0, config_value<double>(cfg, "waypoint_decimation_deg",
                                         profile.waypoint_decimation_deg));
  profile.min_spline_waypoints = std::max<std::size_t>(
      2, config_value<std::size_t>(cfg, "min_spline_waypoints",
                                   profile.min_spline_waypoints));
  profile.max_spline_waypoints =
      std::max(profile.min_spline_waypoints,
               config_value<std::size_t>(cfg, "max_spline_waypoints",
                                         profile.max_spline_waypoints));
  profile.min_segment_seconds =
      std::max(0.001, config_value<double>(cfg, "min_segment_seconds",
                                           profile.min_segment_seconds));
  profile.temp_warn_celsius = config_value<int16_t>(cfg, "temp_warn_celsius",
                                                    profile.temp_warn_celsius);
  profile.temp_abort_celsius =
      std::max(profile.temp_warn_celsius,
               config_value<int16_t>(cfg, "temp_abort_celsius",
                                     profile.temp_abort_celsius));
  profile.log_root_dir =
      config_value<std::string>(cfg, "log_root_dir", profile.log_root_dir);
  profile.log_file_buffer_bytes = std::max<std::size_t>(
      4096, config_value<std::size_t>(cfg, "log_file_buffer_bytes",
                                      profile.log_file_buffer_bytes));
  profile.enable_realtime_scheduler = config_value<bool>(
      cfg, "enable_realtime_scheduler", profile.enable_realtime_scheduler);
  profile.realtime_priority = std::clamp(
      config_value<int>(cfg, "realtime_priority", profile.realtime_priority), 1,
      99);

  return profile;
}

} // namespace actuator_test

// ---- safety.cpp -------------------------------------------------------------
namespace actuator_test {

bool safety_violated(const DriverAdapter &drv, const RuntimeProfile &profile,
                     const std::string &joint_name, SafetyState &state,
                     std::string &reason) {
  if (drv.has_temperature_feedback()) {
    const int16_t motor_t = drv.motor_temperature();
    const int16_t drive_t = drv.drive_temperature();
    const int16_t hottest = std::max(motor_t, drive_t);

    if (hottest >= profile.temp_abort_celsius) {
      reason = "over-temperature (motor=" + std::to_string(motor_t) +
               "C, drive=" + std::to_string(drive_t) + "C)";
      return true;
    }

    if (hottest >= profile.temp_warn_celsius) {
      ECWRN("Joint '%s' warm: motor=%dC drive=%dC\n", joint_name.c_str(),
            static_cast<int>(motor_t), static_cast<int>(drive_t));
    }
  }

  if (drv.fault()) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", drv.status());
    reason = std::string("DS402 fault bit set (status=0x") + buf + ")";
    return true;
  }

  const uint16_t err = drv.error_code();
  if (err != 0 && err == state.last_error) {
    char buf[8];
    std::snprintf(buf, sizeof(buf), "%04X", err);
    reason = std::string("error_code latched (0x") + buf + ")";
    return true;
  }

  state.last_error = err;
  return false;
}

} // namespace actuator_test

// ---- runtime.cpp ------------------------------------------------------------
namespace {

constexpr unsigned k_cap_net_admin = 12U;
constexpr unsigned k_cap_net_raw = 13U;
constexpr unsigned k_cap_sys_nice = 23U;

bool capability_bit_is_set(unsigned long long mask, unsigned bit) noexcept {
  return (mask & (1ULL << bit)) != 0ULL;
}

void signal_handler(int) { ecp::rt_app_t::instance().terminate(); }

} // namespace

namespace actuator_test {

void register_signal_handlers() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
}

CapabilityStatus query_process_capabilities() {
  CapabilityStatus status;
  std::ifstream status_file("/proc/self/status");
  if (!status_file.is_open()) {
    return status;
  }

  std::string line;
  while (std::getline(status_file, line)) {
    if (line.rfind("CapEff:\t", 0) != 0) {
      continue;
    }

    const std::string hex_mask = line.substr(8);
    unsigned long long mask = 0ULL;
    std::stringstream ss;
    ss << std::hex << hex_mask;
    ss >> mask;

    status.net_admin = capability_bit_is_set(mask, k_cap_net_admin);
    status.net_raw = capability_bit_is_set(mask, k_cap_net_raw);
    status.sys_nice = capability_bit_is_set(mask, k_cap_sys_nice);
    break;
  }

  return status;
}

std::string capability_fix_command(const std::string &executable_path) {
  return "sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice+ep " +
         executable_path;
}

bool ensure_runtime_capabilities(const std::string &executable_path,
                                 const RuntimeProfile &profile) {
  const CapabilityStatus status = query_process_capabilities();
  const bool require_sys_nice = profile.enable_realtime_scheduler;
  if (status.all_satisfied(require_sys_nice)) {
    return true;
  }

  std::fprintf(
      stderr,
      "Missing Linux capabilities required for EtherCAT and RT scheduling.\n");
  std::fprintf(stderr, "  CAP_NET_ADMIN: %s\n",
               status.net_admin ? "OK" : "missing");
  std::fprintf(stderr, "  CAP_NET_RAW:   %s\n",
               status.net_raw ? "OK" : "missing");
  std::fprintf(
      stderr, "  CAP_SYS_NICE:  %s%s\n", status.sys_nice ? "OK" : "missing",
      require_sys_nice ? "" : " (optional: realtime scheduler disabled)");
  std::fprintf(stderr, "Apply them once for the built executable with:\n  %s\n",
               capability_fix_command(executable_path).c_str());
  std::fprintf(stderr,
               "Or use the automated project task: pixi run capabilities\n");
  return false;
}

void try_enable_realtime_scheduler(const RuntimeProfile &profile) {
  if (!profile.enable_realtime_scheduler) {
    std::printf("Realtime scheduler disabled by runtime profile\n");
    return;
  }

  sched_param param{};
  param.sched_priority = profile.realtime_priority;
  if (::sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
    std::printf("Realtime scheduler enabled: SCHED_FIFO priority %d\n",
                param.sched_priority);
    return;
  }

  std::perror("warning: failed to enable SCHED_FIFO");
}

} // namespace actuator_test

// ---- device.cpp -------------------------------------------------------------
namespace actuator_test {

namespace {

void populate_common_fields(JointHandle &jh, const ecp::DeviceConfig &cfg,
                            const std::string &device_name,
                            const std::string &driver_name) {
  jh.name = device_name;
  jh.driver_name = driver_name;
  jh.alias = static_cast<uint16_t>(cfg.get<int>(device_name, "alias", 0));
  jh.model = cfg.get<std::string>(device_name, "model", "");
  jh.operation_mode_name =
      cfg.get<std::string>(device_name, "operation_mode", "");
  if (jh.operation_mode_name.empty()) {
    jh.operation_mode_name = (driver_name == "MT_Device") ? "OP_PVT" : "OP_CSP";
  }

  const auto op = ds402_mode_from_name(jh.operation_mode_name);
  if (!op) {
    jh.selectable = false;
    jh.unavailable_reason =
        "unknown operation_mode='" + jh.operation_mode_name + "'";
    return;
  }

  jh.operation_mode_code = *op;
  jh.pvt_kp = cfg.get<int32_t>(device_name, "PVT_KP", 200);
  jh.pvt_kd = cfg.get<int32_t>(device_name, "PVT_KD", 50);
}

class MyActuatorAdapter final : public DriverAdapter {
public:
  MyActuatorAdapter(std::shared_ptr<ecp::MyActuator::Driver> driver,
                    uint16_t encoder_bits)
      : m_driver(std::move(driver)), m_encoder_bits(encoder_bits) {}

  DriverKind kind() const noexcept override { return DriverKind::MyActuator; }

  const char *kind_name() const noexcept override { return "MT_Device"; }

  int32_t actual_position() const noexcept override {
    return m_driver->actual_position();
  }

  int32_t actual_velocity() const noexcept override {
    return m_driver->actual_velocity();
  }

  uint16_t status() const noexcept override { return m_driver->status(); }

  bool fault() const noexcept override { return m_driver->fault(); }

  uint16_t error_code() const noexcept override {
    return m_driver->error_code();
  }

  bool has_temperature_feedback() const noexcept override { return true; }

  int16_t motor_temperature() const noexcept override {
    return m_driver->motor_temperature();
  }

  int16_t drive_temperature() const noexcept override {
    return m_driver->drive_temperature();
  }

  int8_t operation_mode_display() const noexcept override {
    return m_driver->op_mode_display();
  }

  void set_target_position(int32_t value) noexcept override {
    m_driver->set_target_position(value);
  }

  void set_target_velocity(int32_t value) noexcept override {
    m_driver->set_target_velocity(value);
  }

  void update_operation_mode(int8_t op_mode) noexcept override {
    m_driver->update_operation_mode(op_mode);
  }

  void idle() noexcept override { m_driver->idle(); }

  void apply_runtime_gains(int32_t kp, int32_t kd) noexcept override {
    m_driver->set_pvt_kp(kp);
    m_driver->set_pvt_kd(kd);
  }

  uint16_t encoder_bits() const noexcept override { return m_encoder_bits; }

private:
  std::shared_ptr<ecp::MyActuator::Driver> m_driver;
  uint16_t m_encoder_bits;
};

class NovantaAdapter final : public DriverAdapter {
public:
  NovantaAdapter(std::shared_ptr<ecp::Novanta::Driver> driver,
                 uint16_t encoder_bits)
      : m_driver(std::move(driver)), m_encoder_bits(encoder_bits) {}

  DriverKind kind() const noexcept override { return DriverKind::Novanta; }

  const char *kind_name() const noexcept override { return "CapitanDrv"; }

  int32_t actual_position() const noexcept override {
    return m_driver->actual_position();
  }

  int32_t actual_velocity() const noexcept override {
    return m_driver->actual_velocity();
  }

  uint16_t status() const noexcept override { return m_driver->status(); }

  bool fault() const noexcept override { return m_driver->fault(); }

  uint16_t error_code() const noexcept override { return 0; }

  bool has_temperature_feedback() const noexcept override { return false; }

  int16_t motor_temperature() const noexcept override { return -1; }

  int16_t drive_temperature() const noexcept override { return -1; }

  int8_t operation_mode_display() const noexcept override {
    return m_driver->op_mode_display();
  }

  void set_target_position(int32_t value) noexcept override {
    m_driver->set_target_position(value);
  }

  void set_target_velocity(int32_t value) noexcept override {
    m_driver->set_target_velocity(value);
  }

  void update_operation_mode(int8_t op_mode) noexcept override {
    m_driver->update_operation_mode(op_mode);
  }

  void idle() noexcept override { m_driver->idle(); }

  uint16_t encoder_bits() const noexcept override { return m_encoder_bits; }

private:
  std::shared_ptr<ecp::Novanta::Driver> m_driver;
  uint16_t m_encoder_bits;
};

class MyActuatorFactory final : public DriverFactory {
public:
  std::string_view driver_name() const noexcept override { return "MT_Device"; }

  std::optional<JointHandle>
  create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
         const std::string &device_name) const override {
    auto d = bus.get_device<ecp::MyActuator::Driver>(device_name);
    if (!d) {
      std::cerr << "  skip '" << device_name
                << "': MT_Device not present on bus\n";
      return std::nullopt;
    }

    JointHandle jh;
    populate_common_fields(jh, cfg, device_name, std::string(driver_name()));
    jh.encoder_bits =
        cfg.get<int>(device_name, "encoder_bits", d->encoder_bits());
    jh.driver = std::make_shared<MyActuatorAdapter>(
        d, static_cast<uint16_t>(jh.encoder_bits));

    if (jh.operation_mode_name == "OP_PVT") {
      jh.selectable = true;
    } else if (jh.unavailable_reason.empty()) {
      jh.selectable = false;
      jh.unavailable_reason =
          "operation_mode='" + jh.operation_mode_name +
          "' (MT_Device requires OP_PVT for impedance TxPDO watchdog)";
    }

    return jh;
  }
};

class NovantaFactory final : public DriverFactory {
public:
  std::string_view driver_name() const noexcept override {
    return "CapitanDrv";
  }

  std::optional<JointHandle>
  create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
         const std::string &device_name) const override {
    auto d = bus.get_device<ecp::Novanta::Driver>(device_name);
    if (!d) {
      std::cerr << "  skip '" << device_name
                << "': CapitanDrv not present on bus\n";
      return std::nullopt;
    }

    JointHandle jh;
    populate_common_fields(jh, cfg, device_name, std::string(driver_name()));
    jh.encoder_bits = cfg.get<int>(device_name, "encoder_bits", 13);
    jh.driver = std::make_shared<NovantaAdapter>(
        d, static_cast<uint16_t>(jh.encoder_bits));
    if (jh.unavailable_reason.empty()) {
      jh.selectable = true;
    }

    return jh;
  }
};

const std::vector<std::unique_ptr<DriverFactory>> &driver_factories() {
  static const std::vector<std::unique_ptr<DriverFactory>> factories = [] {
    std::vector<std::unique_ptr<DriverFactory>> out;
    out.push_back(std::make_unique<MyActuatorFactory>());
    out.push_back(std::make_unique<NovantaFactory>());
    return out;
  }();
  return factories;
}

} // namespace

std::optional<int8_t> ds402_mode_from_name(const std::string &name) {
  using namespace ecp::DS402;
  if (name == "OP_PP")
    return OP_PP;
  if (name == "OP_PV")
    return OP_PV;
  if (name == "OP_TQ")
    return OP_TQ;
  if (name == "OP_PVT")
    return OP_PVT;
  if (name == "OP_HOM")
    return OP_HOM;
  if (name == "OP_CSP")
    return OP_CSP;
  if (name == "OP_CSV")
    return OP_CSV;
  if (name == "OP_CST")
    return OP_CST;
  return std::nullopt;
}

const DriverFactory *find_driver_factory(const std::string &driver_name) {
  for (const auto &factory : driver_factories()) {
    if (factory->driver_name() == driver_name) {
      return factory.get();
    }
  }
  return nullptr;
}

std::string supported_driver_names() {
  std::string out;
  for (const auto &factory : driver_factories()) {
    if (!out.empty()) {
      out += ", ";
    }
    out += factory->driver_name();
  }
  return out;
}

std::vector<JointHandle> enumerate_joints(const ecp::EthercatBus &bus,
                                          const ecp::DeviceConfig &cfg) {
  std::vector<JointHandle> out;
  for (const auto &name : cfg.device_names()) {
    const std::string driver_name = cfg.get<std::string>(name, "driver", "");
    const DriverFactory *factory = find_driver_factory(driver_name);
    if (factory == nullptr) {
      continue;
    }
    if (auto jh = factory->create(bus, cfg, name)) {
      out.push_back(std::move(*jh));
    }
  }

  return out;
}

std::vector<std::size_t> pick_joints(const std::vector<JointHandle> &joints) {
  std::printf("\n--- Joint picker ---\n");
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto &j = joints[i];
    std::printf(
        "  [%2zu] %-16s drv=%-10s alias=%-5u model=%-10s mode=%-6s enc=%2dbit "
        "KP=%-6d KD=%-5d  %s\n",
        i, j.name.c_str(), j.driver_name.c_str(), j.alias,
        j.model.empty() ? "(default)" : j.model.c_str(),
        j.operation_mode_name.c_str(), j.encoder_bits, j.pvt_kp, j.pvt_kd,
        j.selectable ? "OK" : ("UNAVAILABLE: " + j.unavailable_reason).c_str());
  }

  std::printf("\nEnter joint index(es) to tune (single '3' or comma-separated "
              "'0,2,5'),\n"
              "or 'q' to quit: ");
  std::fflush(stdout);

  std::string line;
  if (!std::getline(std::cin, line)) {
    return {};
  }
  if (line == "q" || line == "Q") {
    return {};
  }

  std::vector<std::size_t> picks;
  std::string token;
  bool bad = false;

  auto flush_token = [&](bool &local_bad) {
    std::size_t a = token.find_first_not_of(" \t\r\n");
    std::size_t b = token.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) {
      token.clear();
      return;
    }
    std::string trimmed = token.substr(a, b - a + 1);
    token.clear();

    try {
      const int idx = std::stoi(trimmed);
      if (idx < 0 || static_cast<std::size_t>(idx) >= joints.size()) {
        std::cerr << "Index '" << trimmed << "' out of range\n";
        local_bad = true;
        return;
      }
      if (!joints[static_cast<std::size_t>(idx)].selectable) {
        std::cerr << "Joint '" << joints[static_cast<std::size_t>(idx)].name
                  << "' is not selectable: "
                  << joints[static_cast<std::size_t>(idx)].unavailable_reason
                  << "\n";
        local_bad = true;
        return;
      }
      for (std::size_t existing : picks) {
        if (existing == static_cast<std::size_t>(idx)) {
          std::cerr << "Duplicate index '" << trimmed << "' in selection\n";
          local_bad = true;
          return;
        }
      }
      picks.push_back(static_cast<std::size_t>(idx));
    } catch (...) {
      std::cerr << "Invalid token: '" << trimmed << "'\n";
      local_bad = true;
    }
  };

  for (char ch : line) {
    if (ch == ',') {
      flush_token(bad);
    } else {
      token.push_back(ch);
    }
  }
  flush_token(bad);

  if (bad || picks.empty()) {
    return {static_cast<std::size_t>(-1)};
  }
  return picks;
}

std::optional<TrajectoryMode> pick_mode(TrajectoryMode previous) {
  std::printf("\n--- Trajectory mode ---\n");
  std::printf("  [s] sin       -- fixed-frequency sinusoid about the captured "
              "mid-point\n");
  std::printf(
      "  [l] chirp-lin -- linear frequency sweep f0->f1->f0 (bode/sweep ID)\n");
  std::printf(
      "  [g] chirp-log -- logarithmic frequency sweep (wide-band ID)\n");
  std::printf(
      "  [t] triangle  -- constant-velocity triangle sweep (friction/range)\n");
  std::printf("  [e] step      -- square-wave step response\n");
  std::printf("  [m] multisine -- summed harmonics, Schroeder phasing "
              "(one-shot FRF)\n");
  std::printf("  [p] spline    -- replay a Catmull-Rom spline through "
              "waypoints you record\n");
  std::printf("                   by backdriving the joint(s) during idle\n");
  std::printf("\nEnter a key (Enter = keep previous '%s'), or 'q' to quit: ",
              mode_to_string(previous));
  std::fflush(stdout);

  std::string line;
  if (!std::getline(std::cin, line)) {
    return std::nullopt;
  }

  std::size_t a = line.find_first_not_of(" \t\r\n");
  std::size_t b = line.find_last_not_of(" \t\r\n");
  if (a == std::string::npos) {
    return previous;
  }

  std::string trimmed = line.substr(a, b - a + 1);
  if (trimmed == "q" || trimmed == "Q") {
    return std::nullopt;
  }
  if (trimmed == "s" || trimmed == "S" || trimmed == "sin") {
    return TrajectoryMode::Sin;
  }
  if (trimmed == "l" || trimmed == "L" || trimmed == "chirp-linear") {
    return TrajectoryMode::ChirpLinear;
  }
  if (trimmed == "g" || trimmed == "G" || trimmed == "chirp-log") {
    return TrajectoryMode::ChirpLog;
  }
  if (trimmed == "t" || trimmed == "T" || trimmed == "triangle") {
    return TrajectoryMode::Triangle;
  }
  if (trimmed == "e" || trimmed == "E" || trimmed == "step") {
    return TrajectoryMode::Step;
  }
  if (trimmed == "m" || trimmed == "M" || trimmed == "multisine") {
    return TrajectoryMode::Multisine;
  }
  if (trimmed == "p" || trimmed == "P" || trimmed == "spline") {
    return TrajectoryMode::Spline;
  }

  std::cerr << "Unrecognised mode '" << trimmed << "', keeping previous.\n";
  return previous;
}

void idle_all(std::vector<JointHandle> &joints) {
  for (auto &j : joints) {
    if (!j.driver) {
      continue;
    }
    j.driver->set_target_position(j.driver->actual_position());
    j.driver->set_target_velocity(0);
    j.driver->idle();
  }
}

} // namespace actuator_test

// ---- trajectory.cpp ---------------------------------------------------------
namespace actuator_test {

SinTrajectory::SinTrajectory(double centre_counts, double amp_counts,
                             double freq_hz, double safety_factor,
                             int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts), m_freq(freq_hz),
      m_safety_factor(safety_factor), m_enc_bits(encoder_bits) {}

double SinTrajectory::approach_target() const noexcept { return m_centre; }

double SinTrajectory::sample(double t_s) const noexcept {
  return m_centre + m_amp * std::sin(2.0 * k_pi * m_freq * t_s);
}

void SinTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, sin\n";
  os << "# sin_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# sin_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# sin_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# sin_amp_deg, " << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits)
     << "\n";
  os << "# sin_freq_hz, " << m_freq << "\n";
  os << "# traj_safety_factor, " << m_safety_factor << "\n";
}

// ---------------------------------------------------------------------------
//  ChirpTrajectory
// ---------------------------------------------------------------------------

ChirpTrajectory::ChirpTrajectory(double centre_counts, double amp_counts,
                                 double f0_hz, double f1_hz,
                                 double sweep_seconds, bool logarithmic,
                                 int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts), m_f0(std::max(1e-6, f0_hz)),
      m_f1(std::max(1e-6, f1_hz)), m_sweep(std::max(1e-3, sweep_seconds)),
      m_log(logarithmic), m_enc_bits(encoder_bits) {}

double ChirpTrajectory::approach_target() const noexcept {
  // sin(phase(0)) = sin(0) = 0, so the chirp starts at the centre.
  return m_centre;
}

double ChirpTrajectory::sweep_integral(double x) const noexcept {
  // F(x) = integral over [0, x] of the up-sweep instantaneous frequency.
  const double T = m_sweep;
  if (!m_log) {
    return m_f0 * x + (m_f1 - m_f0) * x * x / (2.0 * T);
  }

  const double r = m_f1 / m_f0;
  if (std::fabs(r - 1.0) < 1e-9) {
    return m_f0 * x;
  }
  return m_f0 * T / std::log(r) * (std::pow(r, x / T) - 1.0);
}

double ChirpTrajectory::phase_at(double t_s) const noexcept {
  const double T = m_sweep;
  const double period = 2.0 * T;
  double t = t_s;
  if (t < 0.0) {
    t = 0.0;
  }

  const double n = std::floor(t / period);
  const double u = t - n * period;

  const double F_T = sweep_integral(T);
  const double phase_period = 2.0 * k_pi * 2.0 * F_T; // up + mirrored down.

  double partial = 0.0;
  if (u <= T) {
    partial = 2.0 * k_pi * sweep_integral(u);
  } else {
    const double y = u - T; // [0, T] into the mirrored down-sweep.
    partial = 2.0 * k_pi * (2.0 * F_T - sweep_integral(T - y));
  }

  return n * phase_period + partial;
}

double ChirpTrajectory::sample(double t_s) const noexcept {
  return m_centre + m_amp * std::sin(phase_at(t_s));
}

void ChirpTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, " << (m_log ? "chirp_log" : "chirp_linear") << "\n";
  os << "# chirp_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# chirp_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# chirp_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# chirp_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# chirp_f0_hz, " << m_f0 << "\n";
  os << "# chirp_f1_hz, " << m_f1 << "\n";
  os << "# chirp_sweep_seconds, " << m_sweep << "\n";
  os << "# chirp_full_period_s, " << (2.0 * m_sweep) << "\n";
}

// ---------------------------------------------------------------------------
//  TriangleTrajectory
// ---------------------------------------------------------------------------

TriangleTrajectory::TriangleTrajectory(double centre_counts, double amp_counts,
                                       double cycle_seconds,
                                       int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts),
      m_cycle(std::max(1e-3, cycle_seconds)), m_enc_bits(encoder_bits) {}

double TriangleTrajectory::approach_target() const noexcept { return m_centre; }

double TriangleTrajectory::sample(double t_s) const noexcept {
  // (2/pi) * asin(sin(theta)) is a unit triangle starting at 0 and rising.
  const double theta = 2.0 * k_pi * t_s / m_cycle;
  const double tri = (2.0 / k_pi) * std::asin(std::sin(theta));
  return m_centre + m_amp * tri;
}

void TriangleTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, triangle\n";
  os << "# triangle_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# triangle_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# triangle_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# triangle_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# triangle_cycle_seconds, " << m_cycle << "\n";
}

// ---------------------------------------------------------------------------
//  StepTrajectory
// ---------------------------------------------------------------------------

StepTrajectory::StepTrajectory(double centre_counts, double amp_counts,
                               double cycle_seconds, int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts),
      m_cycle(std::max(1e-3, cycle_seconds)), m_enc_bits(encoder_bits) {}

double StepTrajectory::approach_target() const noexcept {
  // First half-cycle commands +amp; ramp the approach there to avoid a step
  // at engage (the LPF still shapes subsequent edges).
  return m_centre + m_amp;
}

double StepTrajectory::sample(double t_s) const noexcept {
  const double phase = std::fmod(t_s, m_cycle) / m_cycle; // [0, 1)
  const double sign = (phase < 0.5) ? 1.0 : -1.0;
  return m_centre + m_amp * sign;
}

void StepTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, step\n";
  os << "# step_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# step_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# step_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# step_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# step_cycle_seconds, " << m_cycle << "\n";
}

// ---------------------------------------------------------------------------
//  MultisineTrajectory
// ---------------------------------------------------------------------------

MultisineTrajectory::MultisineTrajectory(double centre_counts,
                                         double amp_counts, double base_freq_hz,
                                         int harmonics, int encoder_bits)
    : m_centre(centre_counts), m_amp(amp_counts),
      m_base_freq(std::max(1e-6, base_freq_hz)),
      m_harmonics(std::max(1, harmonics)), m_enc_bits(encoder_bits) {
  // Schroeder phases keep the crest factor low so the summed amplitude stays
  // within the captured envelope.
  m_phases.resize(static_cast<std::size_t>(m_harmonics));
  for (int k = 1; k <= m_harmonics; ++k) {
    m_phases[static_cast<std::size_t>(k - 1)] =
        -k_pi * static_cast<double>(k) * static_cast<double>(k - 1) /
        static_cast<double>(m_harmonics);
  }
}

double MultisineTrajectory::approach_target() const noexcept {
  return sample(0.0);
}

double MultisineTrajectory::sample(double t_s) const noexcept {
  double acc = 0.0;
  for (int k = 1; k <= m_harmonics; ++k) {
    const double f = m_base_freq * static_cast<double>(k);
    acc += std::sin(2.0 * k_pi * f * t_s +
                    m_phases[static_cast<std::size_t>(k - 1)]);
  }
  return m_centre + (m_amp / static_cast<double>(m_harmonics)) * acc;
}

void MultisineTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, multisine\n";
  os << "# multisine_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# multisine_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# multisine_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# multisine_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# multisine_base_freq_hz, " << m_base_freq << "\n";
  os << "# multisine_harmonics, " << m_harmonics << "\n";
}

std::unique_ptr<Trajectory>
make_parametric_trajectory(TrajectoryMode mode, double centre, double amp,
                           const RuntimeProfile &profile, int encoder_bits) {
  switch (mode) {
  case TrajectoryMode::Sin:
    return std::make_unique<SinTrajectory>(centre, amp, profile.traj_freq_hz,
                                           profile.traj_safety_factor,
                                           encoder_bits);
  case TrajectoryMode::ChirpLinear:
    return std::make_unique<ChirpTrajectory>(
        centre, amp, profile.chirp_f0_hz, profile.chirp_f1_hz,
        profile.chirp_sweep_seconds, false, encoder_bits);
  case TrajectoryMode::ChirpLog:
    return std::make_unique<ChirpTrajectory>(
        centre, amp, profile.chirp_f0_hz, profile.chirp_f1_hz,
        profile.chirp_sweep_seconds, true, encoder_bits);
  case TrajectoryMode::Triangle:
    return std::make_unique<TriangleTrajectory>(
        centre, amp, profile.triangle_cycle_seconds, encoder_bits);
  case TrajectoryMode::Step:
    return std::make_unique<StepTrajectory>(
        centre, amp, profile.step_cycle_seconds, encoder_bits);
  case TrajectoryMode::Multisine:
    return std::make_unique<MultisineTrajectory>(
        centre, amp, profile.multisine_base_hz, profile.multisine_harmonics,
        encoder_bits);
  case TrajectoryMode::Spline:
    break; // Spline is built from recorded waypoints, not here.
  }
  return std::make_unique<SinTrajectory>(centre, amp, profile.traj_freq_hz,
                                         profile.traj_safety_factor,
                                         encoder_bits);
}

SplineTrajectory::SplineTrajectory(std::vector<double> knot_times,
                                   std::vector<int32_t> waypoint_counts,
                                   int encoder_bits)
    : m_t(std::move(knot_times)),
      m_w(waypoint_counts.begin(), waypoint_counts.end()),
      m_enc_bits(encoder_bits) {
  const std::size_t n = m_w.size();
  m_T = (n > 0) ? m_t.back() : 0.0;
  m_m.assign(n, 0.0);
  if (n >= 2) {
    m_m.front() = (m_w[1] - m_w[0]) / (m_t[1] - m_t[0]);
    m_m.back() = (m_w[n - 1] - m_w[n - 2]) / (m_t[n - 1] - m_t[n - 2]);
    for (std::size_t i = 1; i + 1 < n; ++i) {
      m_m[i] = (m_w[i + 1] - m_w[i - 1]) / (m_t[i + 1] - m_t[i - 1]);
    }
  }
}

double SplineTrajectory::approach_target() const noexcept {
  return m_w.empty() ? 0.0 : m_w.front();
}

double SplineTrajectory::sample(double t_s) const noexcept {
  if (m_w.empty()) {
    return 0.0;
  }
  if (m_w.size() == 1 || m_T <= 0.0) {
    return m_w.front();
  }

  const double period = 2.0 * m_T;
  double u = std::fmod(t_s, period);
  if (u < 0.0) {
    u += period;
  }
  const double s = (u <= m_T) ? u : (period - u);

  std::size_t i = 0;
  if (s >= m_t.back()) {
    i = m_w.size() - 2;
  } else {
    while (i + 1 < m_t.size() && s >= m_t[i + 1]) {
      ++i;
    }
  }

  const double dt = m_t[i + 1] - m_t[i];
  const double tau = (dt > 0.0) ? (s - m_t[i]) / dt : 0.0;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double h00 = 2.0 * tau3 - 3.0 * tau2 + 1.0;
  const double h10 = tau3 - 2.0 * tau2 + tau;
  const double h01 = -2.0 * tau3 + 3.0 * tau2;
  const double h11 = tau3 - tau2;

  return h00 * m_w[i] + h10 * dt * m_m[i] + h01 * m_w[i + 1] +
         h11 * dt * m_m[i + 1];
}

void SplineTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, spline\n";
  os << "# spline_kind, cardinal_catmull_rom\n";
  os << "# spline_waypoint_count, " << m_w.size() << "\n";
  os << "# spline_total_period_s, " << (2.0 * m_T) << "\n";
  os << "# spline_one_way_seconds, " << m_T << "\n";
  os << "# spline_knot_times_s,";
  for (double t : m_t) {
    os << ' ' << t;
  }
  os << "\n# spline_waypoint_counts,";
  for (double w : m_w) {
    os << ' ' << static_cast<int64_t>(w);
  }
  os << "\n# spline_waypoint_deg,";
  for (double w : m_w) {
    os << ' ' << counts2deg(static_cast<int32_t>(w), m_enc_bits);
  }
  os << '\n';
}

double SplineTrajectory::total_one_way_seconds() const noexcept { return m_T; }

std::size_t SplineTrajectory::waypoint_count() const noexcept {
  return m_w.size();
}

} // namespace actuator_test

// ---- logging.cpp ------------------------------------------------------------
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

// ---- session.cpp ------------------------------------------------------------
namespace actuator_test {

bool capture_limits_multi(std::vector<JointPlan> &plans, TrajectoryMode mode,
                          const RuntimeProfile &profile) {
  using clock = std::chrono::steady_clock;
  if (plans.empty()) {
    return false;
  }

  std::printf("\n--- Capture mechanical limits for %zu joint%s (mode=%s) ---\n",
              plans.size(), plans.size() == 1 ? "" : "s", mode_to_string(mode));
  std::printf("All selected drives are IDLE; you should be able to backdrive "
              "each by hand.\n");

  if (mode == TrajectoryMode::Sin) {
    std::printf("Move EACH joint through its FULL range of motion (any order, "
                "any speed).\n");
    std::printf("MIN/MAX are tracked automatically per-joint. Press <Enter> "
                "when ALL joints have\n");
    std::printf("been swept end-to-end, or 'q' to abort.\n\n");
  } else {
    std::printf("Move ALL selected joints together through the trajectory you "
                "want to replay.\n");
    std::printf("Min/max are tracked AND motion is recorded at %.0f Hz for "
                "spline training.\n",
                profile.waypoint_record_rate_hz);
    std::printf(
        "Small dithers are filtered by decimation threshold %.2f deg.\n\n",
        profile.waypoint_decimation_deg);
  }

  for (auto &p : plans) {
    const int32_t seed = p.jh->driver->actual_position();
    p.min_counts = seed;
    p.max_counts = seed;
  }

  struct TrailFrame {
    std::vector<int32_t> pos;
  };

  std::vector<TrailFrame> trail;
  if (mode_requires_recording(mode)) {
    trail.reserve(2048);
    TrailFrame f0;
    f0.pos.reserve(plans.size());
    for (auto &p : plans) {
      f0.pos.push_back(p.jh->driver->actual_position());
    }
    trail.push_back(std::move(f0));
  }

  RawTty tty;
  auto last_print = clock::now() - std::chrono::seconds(2);
  auto last_snap = clock::now();
  bool done = false;
  bool aborted = false;

  while (ecp::rt_app_t::instance().running() && !done && !aborted) {
    int c = tty.try_read();
    while (c >= 0) {
      if (c == 'q' || c == 'Q') {
        aborted = true;
        break;
      }
      if (c == '\n' || c == '\r') {
        done = true;
        break;
      }
      c = tty.try_read();
    }

    for (auto &p : plans) {
      const int32_t a = p.jh->driver->actual_position();
      p.min_counts = std::min(p.min_counts, a);
      p.max_counts = std::max(p.max_counts, a);
    }

    if (mode_requires_recording(mode)) {
      const auto now = clock::now();
      if (std::chrono::duration<double>(now - last_snap).count() >=
          profile.waypoint_record_period_s()) {
        last_snap = now;
        TrailFrame f;
        f.pos.reserve(plans.size());
        for (auto &p : plans) {
          f.pos.push_back(p.jh->driver->actual_position());
        }
        trail.push_back(std::move(f));
      }
    }

    const auto now = clock::now();
    if (std::chrono::duration<double>(now - last_print).count() >=
        profile.status_print_interval_s) {
      last_print = now;
      std::printf("\n  -- live limits%s --\n",
                  mode_requires_recording(mode) ? " + trail" : "");
      for (auto &p : plans) {
        const int32_t a = p.jh->driver->actual_position();
        const double range_deg = counts2deg(p.max_counts - p.min_counts,
                                            p.jh->driver->encoder_bits());
        const char *good = (range_deg >= profile.min_range_deg) ? "OK " : "...";
        std::printf(
            "    [%s] %-16s cur=%+8.2f MIN=%+8.2f MAX=%+8.2f range=%6.2f deg\n",
            good, p.jh->name.c_str(),
            counts2deg(a, p.jh->driver->encoder_bits()),
            counts2deg(p.min_counts, p.jh->driver->encoder_bits()),
            counts2deg(p.max_counts, p.jh->driver->encoder_bits()), range_deg);
      }
      if (mode_requires_recording(mode)) {
        std::printf("    raw trail frames so far: %zu (~%.1f s)\n",
                    trail.size(),
                    trail.size() * profile.waypoint_record_period_s());
      }
      std::fflush(stdout);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::printf("\n");
  if (aborted || !done) {
    return false;
  }

  std::printf("--- Captured envelopes ---\n");
  bool too_small_any = false;
  for (auto &p : plans) {
    const double range_deg =
        counts2deg(p.max_counts - p.min_counts, p.jh->driver->encoder_bits());
    std::printf(
        "  %-16s MIN=%+8.2fdeg MAX=%+8.2fdeg range=%6.2fdeg "
        "centre=%+8.2fdeg%s\n",
        p.jh->name.c_str(),
        counts2deg(p.min_counts, p.jh->driver->encoder_bits()),
        counts2deg(p.max_counts, p.jh->driver->encoder_bits()), range_deg,
        counts2deg((p.min_counts + p.max_counts) / 2,
                   p.jh->driver->encoder_bits()),
        range_deg < profile.min_range_deg ? "  <-- BELOW SAFETY FLOOR" : "");
    if (range_deg < profile.min_range_deg) {
      too_small_any = true;
    }
  }

  if (too_small_any) {
    std::fprintf(
        stderr,
        "One or more joints have range < %.2f deg. Aborting selection.\n",
        profile.min_range_deg);
    return false;
  }

  if (mode_requires_recording(mode)) {
    if (trail.size() < 2) {
      std::fprintf(stderr,
                   "Spline trail too short (%zu frames). Aborting selection.\n",
                   trail.size());
      return false;
    }

    std::vector<std::size_t> kept_indices;
    kept_indices.reserve(trail.size());
    kept_indices.push_back(0);
    for (std::size_t i = 1; i + 1 < trail.size(); ++i) {
      bool moved = false;
      const std::size_t last = kept_indices.back();
      for (std::size_t pi = 0; pi < plans.size(); ++pi) {
        const double d_deg =
            std::fabs(counts2deg(trail[i].pos[pi] - trail[last].pos[pi],
                                 plans[pi].jh->driver->encoder_bits()));
        if (d_deg >= profile.waypoint_decimation_deg) {
          moved = true;
          break;
        }
      }
      if (moved) {
        kept_indices.push_back(i);
      }
    }

    if (kept_indices.back() != trail.size() - 1) {
      bool moved_from_last = false;
      const std::size_t last = kept_indices.back();
      for (std::size_t pi = 0; pi < plans.size(); ++pi) {
        const double d_deg =
            std::fabs(counts2deg(trail.back().pos[pi] - trail[last].pos[pi],
                                 plans[pi].jh->driver->encoder_bits()));
        if (d_deg >= profile.waypoint_decimation_deg) {
          moved_from_last = true;
          break;
        }
      }
      if (moved_from_last) {
        kept_indices.push_back(trail.size() - 1);
      } else {
        kept_indices.back() = trail.size() - 1;
      }
    }

    if (kept_indices.size() > profile.max_spline_waypoints) {
      std::vector<std::size_t> thinned;
      thinned.reserve(profile.max_spline_waypoints);
      const double step = static_cast<double>(kept_indices.size() - 1) /
                          static_cast<double>(profile.max_spline_waypoints - 1);
      for (std::size_t k = 0; k < profile.max_spline_waypoints; ++k) {
        const std::size_t idx = static_cast<std::size_t>(std::round(k * step));
        const std::size_t src = std::min(kept_indices.size() - 1, idx);
        if (thinned.empty() || thinned.back() != kept_indices[src]) {
          thinned.push_back(kept_indices[src]);
        }
      }
      kept_indices.swap(thinned);
    }

    if (kept_indices.size() < profile.min_spline_waypoints) {
      std::fprintf(
          stderr,
          "Spline has only %zu waypoint(s) after decimation (need >= %zu). "
          "Move the joints more, or use sin mode. Aborting selection.\n",
          kept_indices.size(), profile.min_spline_waypoints);
      return false;
    }

    std::printf(
        "--- Recorded spline (%zu waypoints from %zu trail frames) ---\n",
        kept_indices.size(), trail.size());
    for (std::size_t pi = 0; pi < plans.size(); ++pi) {
      plans[pi].waypoint_counts.clear();
      plans[pi].waypoint_counts.reserve(kept_indices.size());
      for (std::size_t k : kept_indices) {
        plans[pi].waypoint_counts.push_back(trail[k].pos[pi]);
      }
    }
  }

  return true;
}

void run_trajectory_multi(std::vector<JointPlan> &plans, TrajectoryMode mode,
                          const RuntimeProfile &profile) {
  using clock = std::chrono::steady_clock;
  if (plans.empty()) {
    return;
  }

  const std::string log_dir = make_run_log_dir(profile);

  if (mode_requires_recording(mode)) {
    const std::size_t n = plans.front().waypoint_counts.size();
    for (const auto &p : plans) {
      if (p.waypoint_counts.size() != n || n < profile.min_spline_waypoints) {
        std::fprintf(stderr,
                     "internal error: spline waypoint mismatch (joint '%s' has "
                     "%zu, expected %zu, min %zu).\n",
                     p.jh->name.c_str(), p.waypoint_counts.size(), n,
                     profile.min_spline_waypoints);
        return;
      }
    }

    std::vector<double> knot_times(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) {
      double seg = 0.0;
      for (const auto &p : plans) {
        const double d_deg = std::fabs(
            counts2deg(p.waypoint_counts[i] - p.waypoint_counts[i - 1],
                       p.jh->driver->encoder_bits()));
        seg = std::max(seg, d_deg / profile.max_approach_speed_deg_s);
      }
      seg = std::max(seg, profile.min_segment_seconds);
      knot_times[i] = knot_times[i - 1] + seg;
    }

    for (auto &p : plans) {
      p.traj = std::make_unique<SplineTrajectory>(knot_times, p.waypoint_counts,
                                                  p.jh->driver->encoder_bits());
    }
  } else {
    for (auto &p : plans) {
      const double centre =
          static_cast<double>((p.min_counts + p.max_counts) / 2);
      const int32_t half_range = (p.max_counts - p.min_counts) / 2;
      const double amp =
          profile.traj_safety_factor * static_cast<double>(half_range);
      p.traj = make_parametric_trajectory(mode, centre, amp, profile,
                                          p.jh->driver->encoder_bits());
    }
  }

  std::printf("\n--- Trajectory plan (%zu joint%s, mode=%s, %.0fHz loop / "
              "%.1fHz LPF) ---\n",
              plans.size(), plans.size() == 1 ? "" : "s", mode_to_string(mode),
              profile.loop_rate_hz, profile.lpf_cutoff_hz);
  for (auto &p : plans) {
    const double target_deg =
        counts2deg(static_cast<int32_t>(p.traj->approach_target()),
                   p.jh->driver->encoder_bits());
    const int32_t preview_counts = p.jh->driver->actual_position();
    const double preview_deg =
        counts2deg(preview_counts, p.jh->driver->encoder_bits());
    const double preview_dist = std::fabs(target_deg - preview_deg);
    const double preview_t =
        std::max(profile.approach_seconds,
                 preview_dist / profile.max_approach_speed_deg_s);

    if (mode == TrajectoryMode::Sin) {
      const double centre_deg = counts2deg((p.min_counts + p.max_counts) / 2,
                                           p.jh->driver->encoder_bits());
      const double amp_deg =
          counts2deg(static_cast<int32_t>(profile.traj_safety_factor *
                                          ((p.max_counts - p.min_counts) / 2)),
                     p.jh->driver->encoder_bits());
      std::printf(
          "  %-16s drv=%-10s mode=%-6s current=%+7.2fdeg centre=%+7.2fdeg "
          "amp=%.2fdeg @ %.2fHz approach~%.2fs KP=%d KD=%d\n",
          p.jh->name.c_str(), p.jh->driver_name.c_str(),
          p.jh->operation_mode_name.c_str(), preview_deg, centre_deg, amp_deg,
          profile.traj_freq_hz, preview_t, p.jh->pvt_kp, p.jh->pvt_kd);
    } else {
      auto *sp = static_cast<SplineTrajectory *>(p.traj.get());
      std::printf("  %-16s drv=%-10s mode=%-6s current=%+7.2fdeg w0=%+7.2fdeg "
                  "waypoints=%zu one-way=%.2fs approach~%.2fs KP=%d KD=%d\n",
                  p.jh->name.c_str(), p.jh->driver_name.c_str(),
                  p.jh->operation_mode_name.c_str(), preview_deg, target_deg,
                  sp->waypoint_count(), sp->total_one_way_seconds(), preview_t,
                  p.jh->pvt_kp, p.jh->pvt_kd);
    }
  }

  std::printf("  loop rate = %.0f Hz, LPF cutoff = %.1f Hz, pre-hold = %.2fs, "
              "max approach speed = %.1f deg/s\n",
              profile.loop_rate_hz, profile.lpf_cutoff_hz,
              profile.pre_ramp_hold_seconds, profile.max_approach_speed_deg_s);
  std::printf("\nPress <Enter> to engage all drives and start, or 'q'+<Enter> "
              "to abort: ");
  std::fflush(stdout);

  {
    std::string line;
    if (!std::getline(std::cin, line)) {
      return;
    }
    if (line == "q" || line == "Q") {
      return;
    }
  }

  for (auto &p : plans) {
    auto &d = *p.jh->driver;
    d.set_target_velocity(0);
    d.set_target_position(d.actual_position());
    d.apply_runtime_gains(p.jh->pvt_kp, p.jh->pvt_kd);
    d.update_operation_mode(p.jh->operation_mode_code);
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (auto &p : plans) {
    p.start_counts = p.jh->driver->actual_position();
    const double start_deg =
        counts2deg(p.start_counts, p.jh->driver->encoder_bits());
    const double target_deg =
        counts2deg(static_cast<int32_t>(p.traj->approach_target()),
                   p.jh->driver->encoder_bits());
    const double dist_deg = std::fabs(target_deg - start_deg);
    p.approach_T = std::max(profile.approach_seconds,
                            dist_deg / profile.max_approach_speed_deg_s);
    p.phase = ControlPhase::Hold;
    p.phase_t = 0.0;
    p.lpf = std::make_unique<LowPass>(profile.lpf_cutoff_hz,
                                      profile.loop_period_s(),
                                      static_cast<double>(p.start_counts));

    std::printf("  engaged '%s': start=%+.2fdeg, ramping to traj "
                "start=%+.2fdeg over %.2fs\n",
                p.jh->name.c_str(), start_deg, target_deg, p.approach_T);

    if (!log_dir.empty()) {
      const std::string path = log_dir + "/" + p.jh->name + ".csv";
      p.csv_logger = std::make_unique<JointCsvLogger>();
      if (p.csv_logger->open(path, profile, *p.jh, p, *p.traj)) {
        std::printf("    logging '%s' to %s\n", p.jh->name.c_str(),
                    path.c_str());
      } else {
        p.csv_logger.reset();
        std::fprintf(
            stderr,
            "warning: cannot open log file '%s' -- skipping log for %s\n",
            path.c_str(), p.jh->name.c_str());
      }
    }
  }

  RawTty tty;
  const auto t0 = clock::now();
  int print_div = 0;
  bool aborted = false;
  std::string abort_reason;
  std::string abort_joint;

  std::printf(
      "\nRunning at %.0f Hz. Press 'q' to stop, '<Enter>' to print status.\n",
      profile.loop_rate_hz);

  int64_t tick = 0;
  while (ecp::rt_app_t::instance().running()) {
    int c = tty.try_read();
    while (c >= 0) {
      if (c == 'q' || c == 'Q') {
        std::printf("\nStop requested.\n");
        aborted = true;
        abort_reason = "user quit";
      } else if (c == '\n' || c == '\r') {
        print_div = 0;
      }
      c = tty.try_read();
    }
    if (aborted) {
      break;
    }

    for (auto &p : plans) {
      std::string reason;
      if (safety_violated(*p.jh->driver, profile, p.jh->name, p.safety,
                          reason)) {
        ECERR("Safety abort on '%s': %s\n", p.jh->name.c_str(), reason.c_str());
        aborted = true;
        abort_reason = reason;
        abort_joint = p.jh->name;
        break;
      }
    }
    if (aborted) {
      break;
    }

    for (auto &p : plans) {
      double ref_counts = 0.0;
      if (p.phase == ControlPhase::Hold) {
        ref_counts = static_cast<double>(p.start_counts);
        if (p.phase_t >= profile.pre_ramp_hold_seconds) {
          p.phase = ControlPhase::Approach;
          p.phase_t = 0.0;
          std::printf("\n[%s] hold complete -- starting min-jerk approach.\n",
                      p.jh->name.c_str());
        }
      } else if (p.phase == ControlPhase::Approach) {
        ref_counts = min_jerk(p.phase_t, p.approach_T,
                              static_cast<double>(p.start_counts),
                              p.traj->approach_target());
        if (p.phase_t >= p.approach_T) {
          p.phase = ControlPhase::Trajectory;
          p.phase_t = 0.0;
          std::printf("\n[%s] approach complete -- starting trajectory.\n",
                      p.jh->name.c_str());
        }
      } else {
        ref_counts = p.traj->sample(p.phase_t);
      }

      double filtered = p.lpf->step(ref_counts);
      filtered = std::clamp(filtered, static_cast<double>(p.min_counts),
                            static_cast<double>(p.max_counts));

      const int32_t target = static_cast<int32_t>(filtered);
      auto &d = *p.jh->driver;
      const int32_t actual_counts = d.actual_position();
      const int16_t motor_t =
          d.has_temperature_feedback() ? d.motor_temperature() : -1;
      const int16_t drive_t =
          d.has_temperature_feedback() ? d.drive_temperature() : -1;
      const uint16_t err = d.error_code();

      d.apply_runtime_gains(p.jh->pvt_kp, p.jh->pvt_kd);
      d.set_target_position(target);
      d.set_target_velocity(0);

      LogSample sample;
      sample.t_s = static_cast<double>(tick) * profile.loop_period_s();
      sample.phase = static_cast<int>(p.phase);
      sample.phase_t_s = p.phase_t;
      sample.ref_raw_counts = ref_counts;
      sample.ref_filt_counts = filtered;
      sample.actual_counts = actual_counts;
      sample.motor_temp_c = motor_t;
      sample.drive_temp_c = drive_t;
      sample.error_code = err;
      if (p.csv_logger) {
        p.csv_logger->write(sample);
      }

      p.phase_t += profile.loop_period_s();
    }

    if (++print_div >= static_cast<int>(0.5 * profile.loop_rate_hz)) {
      print_div = 0;
      std::printf("[%6.2fs]",
                  std::chrono::duration<double>(clock::now() - t0).count());
      for (auto &p : plans) {
        const double actual_deg = counts2deg(p.jh->driver->actual_position(),
                                             p.jh->driver->encoder_bits());
        if (p.jh->driver->has_temperature_feedback()) {
          std::printf("  %s:ph%d act=%+6.2f motor=%dC", p.jh->name.c_str(),
                      static_cast<int>(p.phase), actual_deg,
                      static_cast<int>(p.jh->driver->motor_temperature()));
        } else {
          std::printf("  %s:ph%d act=%+6.2f", p.jh->name.c_str(),
                      static_cast<int>(p.phase), actual_deg);
        }
      }
      std::printf("\n");
      std::fflush(stdout);
    }

    ++tick;
    std::this_thread::sleep_until(
        t0 + std::chrono::microseconds(
                 static_cast<int64_t>(tick * profile.loop_period_s() * 1e6)));
  }

  for (auto &p : plans) {
    auto &d = *p.jh->driver;
    d.set_target_position(d.actual_position());
    d.set_target_velocity(0);
    d.idle();

    if (p.csv_logger && p.csv_logger->is_open()) {
      const std::string log_path = p.csv_logger->path();
      p.csv_logger->close();
      std::printf("  wrote log: %s\n", log_path.c_str());
    }
    p.csv_logger.reset();
  }

  if (aborted) {
    std::printf("Trajectory aborted (%s%s%s).\n",
                abort_joint.empty() ? "" : abort_joint.c_str(),
                abort_joint.empty() ? "" : ": ", abort_reason.c_str());
  } else {
    std::printf("Trajectory loop exited (rt_app stop).\n");
  }
}

} // namespace actuator_test

// =============================================================================
//  main
// =============================================================================

int main(int argc, char *argv[]) {
  using namespace ecp;
  using namespace actuator_test;

  const std::string cfg_path = (argc > 1)
                                   ? std::string(argv[1])
                                   : std::string("../config/gene-000.toml");
  std::printf("actuator-test: loading '%s'\n", cfg_path.c_str());

  auto opt_cfg = DeviceConfig::from_file(cfg_path);
  if (!opt_cfg) {
    std::cerr << "Failed to parse config: " << cfg_path << "\n";
    return 1;
  }
  auto cfg = std::make_shared<DeviceConfig>(std::move(*opt_cfg));
  const RuntimeProfile profile = load_runtime_profile(*cfg);

  register_signal_handlers();

  const std::string executable_path =
      (argc > 0) ? std::string(argv[0]) : std::string("./actuator-test-spline");
  if (!ensure_runtime_capabilities(executable_path, profile)) {
    return 1;
  }

  if (rt_app_t::instance().init() != 0) {
    std::cerr << "rt_app init failed\n";
    return 1;
  }

  try_enable_realtime_scheduler(profile);

  const std::string ifname = get_ethercat_interface();
  if (ifname.empty()) {
    std::cerr << "No EtherCAT interface found.\n";
    return 1;
  }

  EthercatBus bus;
  auto subdevices = std::make_shared<SubDeviceMap>();
  if (bus.startup(ifname, subdevices, cfg) < 0) {
    std::cerr << "Bus startup failed on '" << ifname << "'\n";
    return 1;
  }

  auto joints = enumerate_joints(bus, *cfg);
  if (joints.empty()) {
    std::cerr << "No supported joints matched on the bus. Supported drivers: "
              << supported_driver_names() << "\n";
    return 1;
  }

  std::printf("runtime profile: loop=%.0fHz lpf=%.1fHz traj=%.2fHz "
              "log_root='%s' rt=%s prio=%d\n",
              profile.loop_rate_hz, profile.lpf_cutoff_hz, profile.traj_freq_hz,
              profile.log_root_dir.c_str(),
              profile.enable_realtime_scheduler ? "on" : "off",
              profile.realtime_priority);

  idle_all(joints);

  TrajectoryMode mode = TrajectoryMode::Sin;
  while (rt_app_t::instance().running()) {
    const std::vector<std::size_t> sel = pick_joints(joints);
    if (sel.empty()) {
      break;
    }
    if (sel.size() == 1 && sel[0] == static_cast<std::size_t>(-1)) {
      continue;
    }

    const std::optional<TrajectoryMode> picked_mode = pick_mode(mode);
    if (!picked_mode) {
      break;
    }
    mode = *picked_mode;

    idle_all(joints);

    std::printf("\n=== Selection: %zu joint%s, mode=%s ===\n", sel.size(),
                sel.size() == 1 ? "" : "s", mode_to_string(mode));
    for (std::size_t idx : sel) {
      std::printf("    - %s (drv=%s alias=%u model=%s mode=%s)\n",
                  joints[idx].name.c_str(), joints[idx].driver_name.c_str(),
                  joints[idx].alias,
                  joints[idx].model.empty() ? "(default)"
                                            : joints[idx].model.c_str(),
                  joints[idx].operation_mode_name.c_str());
    }

    std::vector<JointPlan> plans;
    plans.reserve(sel.size());
    for (std::size_t idx : sel) {
      JointPlan p;
      p.jh = &joints[idx];
      plans.push_back(std::move(p));
    }

    if (!capture_limits_multi(plans, mode, profile)) {
      std::printf("Limit capture aborted -- dropping the whole selection.\n");
      idle_all(joints);
      if (!rt_app_t::instance().running()) {
        break;
      }
      continue;
    }

    if (!rt_app_t::instance().running()) {
      break;
    }

    run_trajectory_multi(plans, mode, profile);
    idle_all(joints);
  }

  idle_all(joints);
  std::printf("actuator-test: done.\n");
  return 0;
}
