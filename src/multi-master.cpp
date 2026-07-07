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

namespace ecp
{
    // Automatically discover and return the first valid EtherCAT interface name, or empty string if none found.
    // This is useful so we don't have to hardcode an interface names whenever we change adapter.
    // Usage:
    //   std::string ifname = ecp::get_ethercat_interface();
    //   returns empty string if no valid EtherCAT interface found.
    inline std::vector<std::string> get_ethercat_interfaces() noexcept
    {
        // discover and get vector of available interfaces.
        const auto interfaces = ecp::discover_interfaces();

        // fallback empty string if no interfaces found.
        if (interfaces.empty())
        {
            return {};
        }

        ecx_contextt ctx;
        memset(&ctx, 0, sizeof(ctx));
        std::vector<std::string> valid_ecat_interfaces;

        // for each interface test if ethercat bus exists.
        for (const auto &iface : interfaces)
        {
            if (ecx_init(&ctx, iface.c_str()) <= 0)
            {
                continue;
            }
            if (ecx_config_init(&ctx) <= 0)
            {
                ecx_close(&ctx);
                continue;
            }

            ecx_close(&ctx);

            // if ethercat bus found, add this interface name to the list.
            valid_ecat_interfaces.push_back(iface);
        }
        return valid_ecat_interfaces;
    }
}
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

    const auto ifnames = get_ethercat_interfaces();
    if (ifnames.empty())
    {
        std::cerr << "No valid EtherCAT interfaces found\n";
        return 1;
    }

    std::printf("Found %zu valid EtherCAT interfaces:\n", ifnames.size());
    for (const auto &ifname : ifnames)
    {
        std::printf("  %s\n", ifname.c_str());
    }

    auto subdevices = std::make_shared<SubDeviceMap>();
    std::vector<EthercatBus> buses;
    for (const auto &ifname : ifnames)
    {
        std::printf("Initializing EtherCAT bus on interface '%s'...\n", ifname.c_str());
        EthercatBus bus;
        if (bus.startup(ifname, subdevices, cfg) < 0)
        {
            std::cerr << "Failed to initialize EtherCAT bus on interface '" << ifname << "'\n";
            continue;
        }
        buses.push_back(std::move(bus));
    }
    std::this_thread::sleep_for(1s); // wait a bit for the bus to stabilize before starting the main loop
}