#pragma once

#include "actuator_test/device.hpp"
#include "actuator_test/logging.hpp"
#include "actuator_test/math.hpp"
#include "actuator_test/safety.hpp"
#include "actuator_test/trajectory.hpp"
#include "actuator_test/types.hpp"

#include <memory>
#include <vector>

namespace actuator_test
{

enum class ControlPhase
{
    Hold = 0,
    Approach = 1,
    Trajectory = 2,
};

struct JointPlan
{
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

bool capture_limits_multi(std::vector<JointPlan> &plans, TrajectoryMode mode, const RuntimeProfile &profile);

void run_trajectory_multi(std::vector<JointPlan> &plans, TrajectoryMode mode, const RuntimeProfile &profile);

} // namespace actuator_test
