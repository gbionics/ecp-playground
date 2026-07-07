#include "actuator_test/runtime.hpp"

#include <ethercat-primer/core>

#include <csignal>
#include <cstdio>
#include <fstream>
#include <sched.h>
#include <sstream>
#include <string>

namespace {

constexpr unsigned k_cap_net_admin = 12U;
constexpr unsigned k_cap_net_raw = 13U;
constexpr unsigned k_cap_sys_nice = 23U;

bool capability_bit_is_set(unsigned long long mask, unsigned bit) noexcept {
  return (mask & (1ULL << bit)) != 0ULL;
}

void signal_handler(int) { ecp::rt_app_t::instance().terminate(); }

} // namespace

namespace actuator_test {

void register_signal_handlers() {
  std::signal(SIGINT, signal_handler);
  std::signal(SIGTERM, signal_handler);
}

CapabilityStatus query_process_capabilities() {
  CapabilityStatus status;
  std::ifstream status_file("/proc/self/status");
  if (!status_file.is_open()) {
    return status;
  }

  std::string line;
  while (std::getline(status_file, line)) {
    if (line.rfind("CapEff:\t", 0) != 0) {
      continue;
    }

    const std::string hex_mask = line.substr(8);
    unsigned long long mask = 0ULL;
    std::stringstream ss;
    ss << std::hex << hex_mask;
    ss >> mask;

    status.net_admin = capability_bit_is_set(mask, k_cap_net_admin);
    status.net_raw = capability_bit_is_set(mask, k_cap_net_raw);
    status.sys_nice = capability_bit_is_set(mask, k_cap_sys_nice);
    break;
  }

  return status;
}

std::string capability_fix_command(const std::string &executable_path) {
  return "sudo setcap cap_net_raw,cap_net_admin,cap_sys_nice+ep " +
         executable_path;
}

bool ensure_runtime_capabilities(const std::string &executable_path,
                                 const RuntimeProfile &profile) {
  const CapabilityStatus status = query_process_capabilities();
  const bool require_sys_nice = profile.enable_realtime_scheduler;
  if (status.all_satisfied(require_sys_nice)) {
    return true;
  }

  std::fprintf(
      stderr,
      "Missing Linux capabilities required for EtherCAT and RT scheduling.\n");
  std::fprintf(stderr, "  CAP_NET_ADMIN: %s\n",
               status.net_admin ? "OK" : "missing");
  std::fprintf(stderr, "  CAP_NET_RAW:   %s\n",
               status.net_raw ? "OK" : "missing");
  std::fprintf(
      stderr, "  CAP_SYS_NICE:  %s%s\n", status.sys_nice ? "OK" : "missing",
      require_sys_nice ? "" : " (optional: realtime scheduler disabled)");
  std::fprintf(stderr, "Apply them once for the built executable with:\n  %s\n",
               capability_fix_command(executable_path).c_str());
  std::fprintf(stderr,
               "Or use the automated project task: pixi run capabilities\n");
  return false;
}

void try_enable_realtime_scheduler(const RuntimeProfile &profile) {
  if (!profile.enable_realtime_scheduler) {
    std::printf("Realtime scheduler disabled by runtime profile\n");
    return;
  }

  sched_param param{};
  param.sched_priority = profile.realtime_priority;
  if (::sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
    std::printf("Realtime scheduler enabled: SCHED_FIFO priority %d\n",
                param.sched_priority);
    return;
  }

  std::perror("warning: failed to enable SCHED_FIFO");
}

} // namespace actuator_test
