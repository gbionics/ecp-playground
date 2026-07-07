#include "actuator_test/safety.hpp"

#include <algorithm>
#include <cstdio>

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
