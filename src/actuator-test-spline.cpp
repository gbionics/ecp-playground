// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/device.hpp"
#include "actuator_test/runtime.hpp"
#include "actuator_test/session.hpp"
#include "actuator_test/settings.hpp"
#include "actuator_test/types.hpp"

#include <ethercat-primer/core>

#include <cstdio>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

int main(int argc, char *argv[])
{
    using namespace ecp;
    using namespace actuator_test;

    const std::string cfg_path = (argc > 1) ? std::string(argv[1]) : std::string("../config/gene-000.toml");
    std::printf("actuator-test: loading '%s'\n", cfg_path.c_str());

    auto opt_cfg = DeviceConfig::from_file(cfg_path);
    if (!opt_cfg)
    {
        std::cerr << "Failed to parse config: " << cfg_path << "\n";
        return 1;
    }
    auto cfg = std::make_shared<DeviceConfig>(std::move(*opt_cfg));
    const RuntimeProfile profile = load_runtime_profile(*cfg);

    register_signal_handlers();

    const std::string executable_path = (argc > 0) ? std::string(argv[0]) : std::string("./actuator-test-spline");
    if (!ensure_runtime_capabilities(executable_path, profile))
    {
        return 1;
    }

    if (rt_app_t::instance().init() != 0)
    {
        std::cerr << "rt_app init failed\n";
        return 1;
    }

    try_enable_realtime_scheduler(profile);

    const std::string ifname = get_ethercat_interface();
    if (ifname.empty())
    {
        std::cerr << "No EtherCAT interface found.\n";
        return 1;
    }

    EthercatBus bus;
    auto subdevices = std::make_shared<SubDeviceMap>();
    if (bus.startup(ifname, subdevices, cfg) < 0)
    {
        std::cerr << "Bus startup failed on '" << ifname << "'\n";
        return 1;
    }

    auto joints = enumerate_joints(bus, *cfg);
    if (joints.empty())
    {
        std::cerr << "No supported joints matched on the bus. Supported drivers: " << supported_driver_names() << "\n";
        return 1;
    }

    std::printf("runtime profile: loop=%.0fHz lpf=%.1fHz traj=%.2fHz log_root='%s' rt=%s prio=%d\n",
                profile.loop_rate_hz, profile.lpf_cutoff_hz, profile.traj_freq_hz, profile.log_root_dir.c_str(),
                profile.enable_realtime_scheduler ? "on" : "off", profile.realtime_priority);

    idle_all(joints);

    TrajectoryMode mode = TrajectoryMode::Sin;
    while (rt_app_t::instance().running())
    {
        const std::vector<std::size_t> sel = pick_joints(joints);
        if (sel.empty())
        {
            break;
        }
        if (sel.size() == 1 && sel[0] == static_cast<std::size_t>(-1))
        {
            continue;
        }

        const std::optional<TrajectoryMode> picked_mode = pick_mode(mode);
        if (!picked_mode)
        {
            break;
        }
        mode = *picked_mode;

        idle_all(joints);

        std::printf("\n=== Selection: %zu joint%s, mode=%s ===\n", sel.size(), sel.size() == 1 ? "" : "s",
                    mode_to_string(mode));
        for (std::size_t idx : sel)
        {
            std::printf("    - %s (drv=%s alias=%u model=%s mode=%s)\n", joints[idx].name.c_str(),
                        joints[idx].driver_name.c_str(), joints[idx].alias,
                        joints[idx].model.empty() ? "(default)" : joints[idx].model.c_str(),
                        joints[idx].operation_mode_name.c_str());
        }

        std::vector<JointPlan> plans;
        plans.reserve(sel.size());
        for (std::size_t idx : sel)
        {
            JointPlan p;
            p.jh = &joints[idx];
            plans.push_back(std::move(p));
        }

        if (!capture_limits_multi(plans, mode, profile))
        {
            std::printf("Limit capture aborted -- dropping the whole selection.\n");
            idle_all(joints);
            if (!rt_app_t::instance().running())
            {
                break;
            }
            continue;
        }

        if (!rt_app_t::instance().running())
        {
            break;
        }

        run_trajectory_multi(plans, mode, profile);
        idle_all(joints);
    }

    idle_all(joints);
    std::printf("actuator-test: done.\n");
    return 0;
}
