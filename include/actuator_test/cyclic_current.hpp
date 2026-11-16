// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include <algorithm>
#include <cmath>

namespace actuator_test {

struct CyclicCurrentProfile {
  double stance_a = 0.0;
  double swing_a = 0.0;
  double period_s = 1.0;
  double stance_fraction = 0.6;
  double transition_s = 0.1;
  double entry_exit_s = 1.0;
  double duration_s = 600.0; ///< Cycling duration, excluding entry/exit ramps.

  bool valid() const {
    return std::isfinite(stance_a) && std::isfinite(swing_a) &&
           std::isfinite(period_s) && period_s > 0.0 &&
           std::isfinite(stance_fraction) && stance_fraction > 0.0 &&
           stance_fraction < 1.0 && std::isfinite(transition_s) &&
           transition_s > 0.0 &&
           transition_s <=
               period_s * std::min(stance_fraction, 1.0 - stance_fraction) &&
           std::isfinite(entry_exit_s) && entry_exit_s > 0.0 &&
           std::isfinite(duration_s) && duration_s > 0.0 &&
           std::isfinite(totalDurationS());
  }

  static double blend(double t, double duration, double from, double to) {
    const double u = std::clamp(t / duration, 0.0, 1.0);
    const double s = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    return from + (to - from) * s;
  }

  double cycleValue(double t) const {
    const double phase = std::fmod(t, period_s);
    const double stance_end = period_s * stance_fraction;
    if (phase < stance_end - transition_s)
      return stance_a;
    if (phase < stance_end)
      return blend(phase - (stance_end - transition_s), transition_s,
                   stance_a, swing_a);
    if (phase < period_s - transition_s)
      return swing_a;
    return blend(phase - (period_s - transition_s), transition_s,
                 swing_a, stance_a);
  }

  double totalDurationS() const { return duration_s + 2.0 * entry_exit_s; }

  double value(double elapsed_s, double initial_a) const {
    if (elapsed_s < entry_exit_s)
      return blend(elapsed_s, entry_exit_s, initial_a, stance_a);
    const double t = elapsed_s - entry_exit_s;
    if (t < duration_s)
      return cycleValue(t);
    if (t < duration_s + entry_exit_s)
      return cycleValue(t) * blend(t - duration_s, entry_exit_s, 1.0, 0.0);
    return 0.0;
  }

  double cycleRmsA() const {
    // Integral of the squared minimum-jerk blend over a unit transition.
    const double delta = swing_a - stance_a;
    const double transition_mean_square =
        stance_a * stance_a + stance_a * delta + (181.0 / 462.0) * delta * delta;
    const double integral =
        (period_s * stance_fraction - transition_s) * stance_a * stance_a +
        (period_s * (1.0 - stance_fraction) - transition_s) * swing_a * swing_a +
        2.0 * transition_s * transition_mean_square;
    return std::sqrt(std::max(0.0, integral / period_s));
  }
};

} // namespace actuator_test
