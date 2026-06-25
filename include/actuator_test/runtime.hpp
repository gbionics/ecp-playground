#pragma once

#include "actuator_test/settings.hpp"

#include <string>

namespace actuator_test
{

struct CapabilityStatus
{
    bool net_admin = false;
    bool net_raw = false;
    bool sys_nice = false;

    bool all_satisfied(bool require_sys_nice = true) const noexcept
    {
        return net_admin && net_raw && (!require_sys_nice || sys_nice);
    }
};

void register_signal_handlers();

CapabilityStatus query_process_capabilities();

std::string capability_fix_command(const std::string &executable_path);

bool ensure_runtime_capabilities(const std::string &executable_path, const RuntimeProfile &profile);

void try_enable_realtime_scheduler(const RuntimeProfile &profile);

} // namespace actuator_test
