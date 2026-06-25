#include "actuator_test/settings.hpp"

#include <ethercat-primer/core>

#include <algorithm>

namespace actuator_test
{

namespace
{

template <typename T>
T config_value(const ecp::DeviceConfig &cfg, const char *key, const T &fallback)
{
    return cfg.get<T>("actuator_test", key, fallback);
}

} // namespace

RuntimeProfile load_runtime_profile(const ecp::DeviceConfig &cfg)
{
    RuntimeProfile profile;

    profile.loop_rate_hz = std::max(1.0, config_value<double>(cfg, "loop_rate_hz", profile.loop_rate_hz));
    profile.lpf_cutoff_hz = std::max(0.01, config_value<double>(cfg, "lpf_cutoff_hz", profile.lpf_cutoff_hz));
    profile.traj_freq_hz = std::max(0.001, config_value<double>(cfg, "traj_freq_hz", profile.traj_freq_hz));
    profile.traj_safety_factor = std::clamp(config_value<double>(cfg, "traj_safety_factor", profile.traj_safety_factor), 0.0, 1.0);
    profile.chirp_f0_hz = std::max(1e-4, config_value<double>(cfg, "chirp_f0_hz", profile.chirp_f0_hz));
    profile.chirp_f1_hz = std::max(1e-4, config_value<double>(cfg, "chirp_f1_hz", profile.chirp_f1_hz));
    profile.chirp_sweep_seconds = std::max(0.1, config_value<double>(cfg, "chirp_sweep_seconds", profile.chirp_sweep_seconds));
    profile.triangle_cycle_seconds = std::max(0.1, config_value<double>(cfg, "triangle_cycle_seconds", profile.triangle_cycle_seconds));
    profile.step_cycle_seconds = std::max(0.1, config_value<double>(cfg, "step_cycle_seconds", profile.step_cycle_seconds));
    profile.multisine_base_hz = std::max(1e-4, config_value<double>(cfg, "multisine_base_hz", profile.multisine_base_hz));
    profile.multisine_harmonics = std::clamp(config_value<int>(cfg, "multisine_harmonics", profile.multisine_harmonics), 1, 64);
    profile.approach_seconds = std::max(0.0, config_value<double>(cfg, "approach_seconds", profile.approach_seconds));
    profile.max_approach_speed_deg_s = std::max(0.1, config_value<double>(cfg, "max_approach_speed_deg_s", profile.max_approach_speed_deg_s));
    profile.pre_ramp_hold_seconds = std::max(0.0, config_value<double>(cfg, "pre_ramp_hold_seconds", profile.pre_ramp_hold_seconds));
    profile.status_print_interval_s = std::max(0.1, config_value<double>(cfg, "status_print_interval_s", profile.status_print_interval_s));
    profile.min_range_deg = std::max(0.0, config_value<double>(cfg, "min_range_deg", profile.min_range_deg));
    profile.waypoint_record_rate_hz = std::max(1.0, config_value<double>(cfg, "waypoint_record_rate_hz", profile.waypoint_record_rate_hz));
    profile.waypoint_decimation_deg = std::max(0.0, config_value<double>(cfg, "waypoint_decimation_deg", profile.waypoint_decimation_deg));
    profile.min_spline_waypoints = std::max<std::size_t>(2, config_value<std::size_t>(cfg, "min_spline_waypoints", profile.min_spline_waypoints));
    profile.max_spline_waypoints = std::max(profile.min_spline_waypoints, config_value<std::size_t>(cfg, "max_spline_waypoints", profile.max_spline_waypoints));
    profile.min_segment_seconds = std::max(0.001, config_value<double>(cfg, "min_segment_seconds", profile.min_segment_seconds));
    profile.temp_warn_celsius = config_value<int16_t>(cfg, "temp_warn_celsius", profile.temp_warn_celsius);
    profile.temp_abort_celsius = std::max(profile.temp_warn_celsius, config_value<int16_t>(cfg, "temp_abort_celsius", profile.temp_abort_celsius));
    profile.log_root_dir = config_value<std::string>(cfg, "log_root_dir", profile.log_root_dir);
    profile.log_file_buffer_bytes = std::max<std::size_t>(4096, config_value<std::size_t>(cfg, "log_file_buffer_bytes", profile.log_file_buffer_bytes));
    profile.enable_realtime_scheduler = config_value<bool>(cfg, "enable_realtime_scheduler", profile.enable_realtime_scheduler);
    profile.realtime_priority = std::clamp(config_value<int>(cfg, "realtime_priority", profile.realtime_priority), 1, 99);

    return profile;
}

} // namespace actuator_test
