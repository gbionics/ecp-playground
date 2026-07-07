#pragma once

#include "actuator_test/device.hpp"
#include "actuator_test/settings.hpp"

#include <cstdint>
#include <string>

namespace actuator_test {

struct SafetyState {
  uint16_t last_error = 0;
};

bool safety_violated(const DriverAdapter &drv, const RuntimeProfile &profile,
                     const std::string &joint_name, SafetyState &state,
                     std::string &reason);

} // namespace actuator_test
