// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/speed_hold.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>

using actuator_test::speed_hold_reaches_limit;
using actuator_test::SpeedHoldReference;
using actuator_test::valid_speed_hold_parameters;

namespace {
void require(bool passed, const char *message) {
  if (!passed) {
    std::cerr << message << '\n';
    std::exit(1);
  }
}
} // namespace

int main() {
  require(valid_speed_hold_parameters(360.0, 1.0, 19, 50),
          "valid positive speed rejected");
  require(valid_speed_hold_parameters(-360.0, 1.0, 19, 50),
          "valid negative speed rejected");
  require(!valid_speed_hold_parameters(0.0, 1.0, 19, 50),
          "zero speed accepted");
  require(!valid_speed_hold_parameters(360.0, 0.0, 19, 50),
          "zero ramp accepted");
  require(!valid_speed_hold_parameters(360.0, -1.0, 19, 50),
          "negative ramp accepted");
  require(!valid_speed_hold_parameters(360.0, 1.0, 19, 0),
          "zero damping accepted");
  require(!valid_speed_hold_parameters(360.0, 1.0, 19, -1),
          "negative damping accepted");
  require(valid_speed_hold_parameters(360.0, 1.0, 19, 0, 20000),
          "position feedback without damping rejected");
  require(valid_speed_hold_parameters(360.0, 1.0, 19, 50, 20000),
          "combined position and velocity feedback rejected");
  require(!valid_speed_hold_parameters(360.0, 1.0, 19, 50, -1),
          "negative stiffness accepted");
  require(!valid_speed_hold_parameters(360.0, 1.0, 0, 50),
          "invalid encoder accepted");
  require(!valid_speed_hold_parameters(360.0, 1.0, 32, 50),
          "oversized encoder accepted");
  require(!valid_speed_hold_parameters(1e20, 1.0, 19, 50),
          "velocity PDO overflow accepted");
  require(!valid_speed_hold_parameters(-1e20, 1.0, 19, 50),
          "negative velocity PDO overflow accepted");
  require(!valid_speed_hold_parameters(1e-12, 1.0, 19, 50),
          "unrepresentable sub-count speed accepted");
  require(!valid_speed_hold_parameters(std::numeric_limits<double>::infinity(),
                                       1.0, 19, 50),
          "infinite speed accepted");
  require(!valid_speed_hold_parameters(
              360.0, std::numeric_limits<double>::quiet_NaN(), 19, 50),
          "NaN ramp accepted");

  require(!speed_hold_reaches_limit(0, 100, 100, -100, 100, 0.001),
          "in-range travel blocked");
  require(speed_hold_reaches_limit(101, -100, -100, -100, 100, 0.001),
          "out-of-range actual position ignored");
  require(speed_hold_reaches_limit(100, 100, 0, -100, 100, 0.001),
          "positive limit ignored");
  require(speed_hold_reaches_limit(-100, -100, 0, -100, 100, 0.001),
          "negative limit ignored");
  require(!speed_hold_reaches_limit(100, -100, -100, -100, 100, 0.001),
          "inward motion from positive limit blocked");
  require(!speed_hold_reaches_limit(-100, 100, 100, -100, 100, 0.001),
          "inward motion from negative limit blocked");
  require(speed_hold_reaches_limit(99, -1, 1000, -100, 100, 0.001),
          "outward measured velocity ignored during reversal");
  require(speed_hold_reaches_limit(-99, 1, -1000, -100, 100, 0.001),
          "negative outward measured velocity ignored during reversal");

  SpeedHoldReference ref;
  ref.reset(360.0, 1.0, 19);
  require(ref.step(0.25) == 131072, "quarter-ramp velocity incorrect");
  require(ref.step(0.25) == 262144, "half-ramp velocity incorrect");
  require(ref.step(0.5) == 524288, "target velocity incorrect");
  require(ref.velocity_deg_s() == 360.0, "reported velocity incorrect");
  for (int n = 0; n < 1000000; ++n) {
    require(ref.step(1000.0) == 524288, "long-running hold drifted");
  }
  require(ref.target_deg_s() == 360.0, "target changed during hold");
  require(ref.position_counts() == 0.0,
          "velocity-only hold accumulated a position reference");

  ref.reset(-360.0, 0.5, 19);
  require(ref.velocity_deg_s() == 0.0, "reset retained old velocity");
  require(ref.step(0.25) == -262144, "negative ramp incorrect");
  require(ref.step(0.25) == -524288, "negative target incorrect");
  require(ref.step(1.0) == -524288, "negative hold incorrect");

  ref.reset(90.0, 0.001, 17);
  require(ref.step(0.01) == 32768, "short ramp overshot");

  ref.reset(360.0, 1.0, 19, 1234, true);
  ref.step(0.25);
  require(ref.position_counts() == 1234 + 16384,
          "quarter ramp position integration incorrect");
  ref.step(0.25);
  require(ref.position_counts() == 1234 + 65536,
          "half ramp position integration incorrect");
  ref.step(0.5);
  require(ref.position_counts() == 1234 + 262144,
          "full ramp position integration incorrect");
  ref.step(1.0);
  require(ref.position_counts() == 1234 + 786432,
          "steady-speed position integration incorrect");
  require(ref.position_safe(1234 + 786432, 524288, 0.001),
          "normal position reference considered unsafe");

  ref.reset(-360.0, 1.0, 19, -1234, true);
  ref.step(1.5);
  require(ref.position_counts() == -1234 - 524288,
          "negative ramp plus steady interval integrated incorrectly");
  ref.reset(360.0, 0.001, 19, 0, true);
  ref.step(0.01);
  require(std::fabs(ref.position_counts() - 524288 * 0.0095) < 1e-9,
          "step crossing end of ramp integrated incorrectly");
  ref.reset(360.0, 1.0, 19, std::numeric_limits<int32_t>::max(), true);
  require(!ref.position_safe(std::numeric_limits<int32_t>::max(), 0, 0.001),
          "positive position rollover not guarded");
  ref.reset(-360.0, 1.0, 19, std::numeric_limits<int32_t>::min(), true);
  require(!ref.position_safe(std::numeric_limits<int32_t>::min(), 0, 0.001),
          "negative position rollover not guarded");
  ref.reset(360.0, 1.0, 19, 0, true);
  require(!ref.position_safe(std::numeric_limits<int32_t>::max(), 0, 0.001),
          "measured position rollover not guarded");
  ref.step(10000);
  require(!ref.position_safe(0, 0, 0.001),
          "out-of-range integrated position not guarded");
  std::cout << "Speed-hold gains, ramp, position and rollover checks passed\n";
}
