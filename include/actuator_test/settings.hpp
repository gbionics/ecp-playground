#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace ecp {

class DeviceConfig;

} // namespace ecp

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
