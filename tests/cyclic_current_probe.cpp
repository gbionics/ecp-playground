// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/cyclic_current.hpp"

#include <cassert>
#include <cmath>
#include <limits>

using actuator_test::CyclicCurrentProfile;

static bool near(double a, double b, double tolerance = 1e-8) {
  return std::fabs(a - b) <= tolerance;
}

int main() {
  CyclicCurrentProfile p;
  p.stance_a = 4.0;
  p.swing_a = -2.0;
  p.duration_s = 2.345;
  assert(p.valid());
  assert(near(p.value(0, 1.5), 1.5));
  assert(near(p.value(p.entry_exit_s, 1.5), p.stance_a));
  assert(near(p.cycleValue(0.25), 4.0));
  assert(near(p.cycleValue(0.55), 1.0));
  assert(near(p.cycleValue(0.75), -2.0));
  assert(near(p.cycleValue(0.95), 1.0));
  assert(near(p.cycleValue(1000.25), 4.0));
  assert(near(p.value(p.totalDurationS(), 1.5), 0.0));
  assert(near(p.value(p.totalDurationS() + 100, 1.5), 0.0));
  const double eps = 1e-7;
  for (double t : {0.5, 0.6, 0.9, 1.0}) {
    assert(near(p.cycleValue(t - eps), p.cycleValue(t + eps)));
    assert(std::fabs((p.cycleValue(t + eps) - p.cycleValue(t)) / eps) < 1e-5);
  }
  for (double t : {p.entry_exit_s, p.entry_exit_s + p.duration_s,
                   p.totalDurationS()})
    assert(near(p.value(t - eps, 1.5), p.value(t + eps, 1.5), 1e-4));
  p.duration_s = 0.55; // End cycling during a transition.
  const double exit = p.entry_exit_s + p.duration_s;
  const double left_slope = (p.value(exit, 0) - p.value(exit - eps, 0)) / eps;
  const double right_slope = (p.value(exit + eps, 0) - p.value(exit, 0)) / eps;
  assert(near(left_slope, right_slope, 0.001));

  double sum_sq = 0.0;
  constexpr int n = 100000;
  for (int i = 0; i < n; ++i) {
    const double t = (i + 0.5) * p.period_s / n;
    const double a = p.cycleValue(t);
    assert(a >= -2.0 - eps && a <= 4.0 + eps);
    sum_sq += a * a;
  }
  assert(near(std::sqrt(sum_sq / n), p.cycleRmsA()));
  p.swing_a = p.stance_a;
  assert(near(p.cycleRmsA(), 4.0));
  p.transition_s = 0.5;
  assert(!p.valid());
  p.transition_s = 0.1;
  p.entry_exit_s = 0.0;
  assert(!p.valid());
  p.entry_exit_s = 1.0;
  p.stance_fraction = 1.0;
  assert(!p.valid());
  p.stance_fraction = 0.6;
  p.period_s = 0.0;
  assert(!p.valid());
  p.period_s = 1.0;
  p.duration_s = std::numeric_limits<double>::infinity();
  assert(!p.valid());
  p.duration_s = 1.0;
  p.stance_a = std::numeric_limits<double>::quiet_NaN();
  assert(!p.valid());
}
