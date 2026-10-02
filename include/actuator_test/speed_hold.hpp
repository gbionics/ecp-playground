// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace actuator_test {

inline bool valid_speed_hold_parameters(double velocity_deg_s,
                                        double ramp_time_s, int encoder_bits,
                                        int32_t damping,
                                        int32_t stiffness = 0) noexcept {
  if (!std::isfinite(velocity_deg_s) || velocity_deg_s == 0.0 ||
      !std::isfinite(ramp_time_s) || ramp_time_s <= 0.0 || encoder_bits < 1 ||
      encoder_bits > 31 || damping < 0 || stiffness < 0 ||
      (damping == 0 && stiffness == 0)) {
    return false;
  }
  const double counts_per_s =
      velocity_deg_s / 360.0 * static_cast<double>(int64_t{1} << encoder_bits);
  return std::fabs(counts_per_s) >= 0.5 &&
         std::fabs(counts_per_s) <
             static_cast<double>(std::numeric_limits<int32_t>::max()) - 0.5;
}

inline bool speed_hold_reaches_limit(int32_t position, int32_t target_velocity,
                                     int32_t actual_velocity, double lo,
                                     double hi, double dt_s) noexcept {
  if (position < lo || position > hi) {
    return true;
  }
  for (const int32_t velocity : {target_velocity, actual_velocity}) {
    const double next =
        static_cast<double>(position) + static_cast<double>(velocity) * dt_s;
    if ((velocity > 0 && next >= hi) || (velocity < 0 && next <= lo)) {
      return true;
    }
  }
  return false;
}

class SpeedHoldReference {
public:
  void reset(double velocity_deg_s, double ramp_time_s, int encoder_bits,
             int32_t initial_position = 0,
             bool track_position = false) noexcept {
    m_target = velocity_deg_s;
    m_ramp_time = ramp_time_s;
    m_elapsed = 0.0;
    m_velocity = 0.0;
    m_counts_per_deg = static_cast<double>(int64_t{1} << encoder_bits) / 360.0;
    m_position = initial_position;
    m_track_position = track_position;
  }

  int32_t step(double dt_s) noexcept {
    const double ramp_dt = std::min(dt_s, m_ramp_time - m_elapsed);
    const double previous_velocity = m_velocity;
    m_elapsed += ramp_dt;
    m_velocity = m_target * (m_elapsed / m_ramp_time);
    if (m_track_position) {
      m_position +=
          m_counts_per_deg * (0.5 * (previous_velocity + m_velocity) * ramp_dt +
                              m_target * (dt_s - ramp_dt));
    }
    return static_cast<int32_t>(std::llround(m_velocity * m_counts_per_deg));
  }

  double velocity_deg_s() const noexcept { return m_velocity; }
  double target_deg_s() const noexcept { return m_target; }
  double position_counts() const noexcept { return m_position; }

  bool position_safe(int32_t actual_position, int32_t actual_velocity,
                     double dt_s) const noexcept {
    // Firmware position-error arithmetic across signed rollover is not
    // specified. Keep one revolution (or one fast tick) clear of that boundary.
    const double margin =
        std::max(m_counts_per_deg * 360.0,
                 std::max(std::fabs(static_cast<double>(actual_velocity)),
                          std::fabs(m_target * m_counts_per_deg)) *
                     dt_s);
    const double lo = std::numeric_limits<int32_t>::min() + margin;
    const double hi = std::numeric_limits<int32_t>::max() - margin;
    return std::isfinite(m_position) && m_position > lo && m_position < hi &&
           actual_position > lo && actual_position < hi &&
           std::fabs(m_position - actual_position) <
               std::numeric_limits<int32_t>::max();
  }

private:
  double m_target = 0.0;
  double m_ramp_time = 1.0;
  double m_elapsed = 0.0;
  double m_velocity = 0.0;
  double m_counts_per_deg = 1.0;
  double m_position = 0.0;
  bool m_track_position = false;
};

} // namespace actuator_test
