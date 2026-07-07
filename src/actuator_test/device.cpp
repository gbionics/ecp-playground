#include "actuator_test/device.hpp"

#include "actuator_test/types.hpp"

#include <cstdio>
#include <iostream>
#include <memory>
#include <string_view>

namespace actuator_test {

namespace {

void populate_common_fields(JointHandle &jh, const ecp::DeviceConfig &cfg,
                            const std::string &device_name,
                            const std::string &driver_name) {
  jh.name = device_name;
  jh.driver_name = driver_name;
  jh.alias = static_cast<uint16_t>(cfg.get<int>(device_name, "alias", 0));
  jh.model = cfg.get<std::string>(device_name, "model", "");
  jh.operation_mode_name =
      cfg.get<std::string>(device_name, "operation_mode", "");
  if (jh.operation_mode_name.empty()) {
    jh.operation_mode_name = (driver_name == "MT_Device") ? "OP_PVT" : "OP_CSP";
  }

  const auto op = ds402_mode_from_name(jh.operation_mode_name);
  if (!op) {
    jh.selectable = false;
    jh.unavailable_reason =
        "unknown operation_mode='" + jh.operation_mode_name + "'";
    return;
  }

  jh.operation_mode_code = *op;
  jh.pvt_kp = cfg.get<int32_t>(device_name, "PVT_KP", 200);
  jh.pvt_kd = cfg.get<int32_t>(device_name, "PVT_KD", 50);
}

class MyActuatorAdapter final : public DriverAdapter {
public:
  MyActuatorAdapter(std::shared_ptr<ecp::MyActuator::Driver> driver,
                    uint16_t encoder_bits)
      : m_driver(std::move(driver)), m_encoder_bits(encoder_bits) {}

  DriverKind kind() const noexcept override { return DriverKind::MyActuator; }

  const char *kind_name() const noexcept override { return "MT_Device"; }

  int32_t actual_position() const noexcept override {
    return m_driver->actual_position();
  }

  int32_t actual_velocity() const noexcept override {
    return m_driver->actual_velocity();
  }

  uint16_t status() const noexcept override { return m_driver->status(); }

  bool fault() const noexcept override { return m_driver->fault(); }

  uint16_t error_code() const noexcept override {
    return m_driver->error_code();
  }

  bool has_temperature_feedback() const noexcept override { return true; }

  int16_t motor_temperature() const noexcept override {
    return m_driver->motor_temperature();
  }

  int16_t drive_temperature() const noexcept override {
    return m_driver->drive_temperature();
  }

  int8_t operation_mode_display() const noexcept override {
    return m_driver->op_mode_display();
  }

  void set_target_position(int32_t value) noexcept override {
    m_driver->set_target_position(value);
  }

  void set_target_velocity(int32_t value) noexcept override {
    m_driver->set_target_velocity(value);
  }

  void update_operation_mode(int8_t op_mode) noexcept override {
    m_driver->update_operation_mode(op_mode);
  }

  void idle() noexcept override { m_driver->idle(); }

  void apply_runtime_gains(int32_t kp, int32_t kd) noexcept override {
    m_driver->set_pvt_kp(kp);
    m_driver->set_pvt_kd(kd);
  }

  uint16_t encoder_bits() const noexcept override { return m_encoder_bits; }

private:
  std::shared_ptr<ecp::MyActuator::Driver> m_driver;
  uint16_t m_encoder_bits;
};

class NovantaAdapter final : public DriverAdapter {
public:
  NovantaAdapter(std::shared_ptr<ecp::Novanta::Driver> driver,
                 uint16_t encoder_bits)
      : m_driver(std::move(driver)), m_encoder_bits(encoder_bits) {}

  DriverKind kind() const noexcept override { return DriverKind::Novanta; }

  const char *kind_name() const noexcept override { return "CapitanDrv"; }

  int32_t actual_position() const noexcept override {
    return m_driver->actual_position();
  }

  int32_t actual_velocity() const noexcept override {
    return m_driver->actual_velocity();
  }

  uint16_t status() const noexcept override { return m_driver->status(); }

  bool fault() const noexcept override { return m_driver->fault(); }

  uint16_t error_code() const noexcept override { return 0; }

  bool has_temperature_feedback() const noexcept override { return false; }

  int16_t motor_temperature() const noexcept override { return -1; }

  int16_t drive_temperature() const noexcept override { return -1; }

  int8_t operation_mode_display() const noexcept override {
    return m_driver->op_mode_display();
  }

  void set_target_position(int32_t value) noexcept override {
    m_driver->set_target_position(value);
  }

  void set_target_velocity(int32_t value) noexcept override {
    m_driver->set_target_velocity(value);
  }

  void update_operation_mode(int8_t op_mode) noexcept override {
    m_driver->update_operation_mode(op_mode);
  }

  void idle() noexcept override { m_driver->idle(); }

  uint16_t encoder_bits() const noexcept override { return m_encoder_bits; }

private:
  std::shared_ptr<ecp::Novanta::Driver> m_driver;
  uint16_t m_encoder_bits;
};

class MyActuatorFactory final : public DriverFactory {
public:
  std::string_view driver_name() const noexcept override { return "MT_Device"; }

  std::optional<JointHandle>
  create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
         const std::string &device_name) const override {
    auto d = bus.get_device<ecp::MyActuator::Driver>(device_name);
    if (!d) {
      std::cerr << "  skip '" << device_name
                << "': MT_Device not present on bus\n";
      return std::nullopt;
    }

    JointHandle jh;
    populate_common_fields(jh, cfg, device_name, std::string(driver_name()));
    jh.encoder_bits =
        cfg.get<int>(device_name, "encoder_bits", d->encoder_bits());
    jh.driver = std::make_shared<MyActuatorAdapter>(
        d, static_cast<uint16_t>(jh.encoder_bits));

    if (jh.operation_mode_name == "OP_PVT") {
      jh.selectable = true;
    } else if (jh.unavailable_reason.empty()) {
      jh.selectable = false;
      jh.unavailable_reason =
          "operation_mode='" + jh.operation_mode_name +
          "' (MT_Device requires OP_PVT for impedance TxPDO watchdog)";
    }

    return jh;
  }
};

class NovantaFactory final : public DriverFactory {
public:
  std::string_view driver_name() const noexcept override {
    return "CapitanDrv";
  }

  std::optional<JointHandle>
  create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
         const std::string &device_name) const override {
    auto d = bus.get_device<ecp::Novanta::Driver>(device_name);
    if (!d) {
      std::cerr << "  skip '" << device_name
                << "': CapitanDrv not present on bus\n";
      return std::nullopt;
    }

    JointHandle jh;
    populate_common_fields(jh, cfg, device_name, std::string(driver_name()));
    jh.encoder_bits = cfg.get<int>(device_name, "encoder_bits", 13);
    jh.driver = std::make_shared<NovantaAdapter>(
        d, static_cast<uint16_t>(jh.encoder_bits));
    if (jh.unavailable_reason.empty()) {
      jh.selectable = true;
    }

    return jh;
  }
};

const std::vector<std::unique_ptr<DriverFactory>> &driver_factories() {
  static const std::vector<std::unique_ptr<DriverFactory>> factories = [] {
    std::vector<std::unique_ptr<DriverFactory>> out;
    out.push_back(std::make_unique<MyActuatorFactory>());
    out.push_back(std::make_unique<NovantaFactory>());
    return out;
  }();
  return factories;
}

} // namespace

std::optional<int8_t> ds402_mode_from_name(const std::string &name) {
  using namespace ecp::DS402;
  if (name == "OP_PP")
    return OP_PP;
  if (name == "OP_PV")
    return OP_PV;
  if (name == "OP_TQ")
    return OP_TQ;
  if (name == "OP_PVT")
    return OP_PVT;
  if (name == "OP_HOM")
    return OP_HOM;
  if (name == "OP_CSP")
    return OP_CSP;
  if (name == "OP_CSV")
    return OP_CSV;
  if (name == "OP_CST")
    return OP_CST;
  return std::nullopt;
}

const DriverFactory *find_driver_factory(const std::string &driver_name) {
  for (const auto &factory : driver_factories()) {
    if (factory->driver_name() == driver_name) {
      return factory.get();
    }
  }
  return nullptr;
}

std::string supported_driver_names() {
  std::string out;
  for (const auto &factory : driver_factories()) {
    if (!out.empty()) {
      out += ", ";
    }
    out += factory->driver_name();
  }
  return out;
}

std::vector<JointHandle> enumerate_joints(const ecp::EthercatBus &bus,
                                          const ecp::DeviceConfig &cfg) {
  std::vector<JointHandle> out;
  for (const auto &name : cfg.device_names()) {
    const std::string driver_name = cfg.get<std::string>(name, "driver", "");
    const DriverFactory *factory = find_driver_factory(driver_name);
    if (factory == nullptr) {
      continue;
    }
    if (auto jh = factory->create(bus, cfg, name)) {
      out.push_back(std::move(*jh));
    }
  }

  return out;
}

std::vector<std::size_t> pick_joints(const std::vector<JointHandle> &joints) {
  std::printf("\n--- Joint picker ---\n");
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const auto &j = joints[i];
    std::printf(
        "  [%2zu] %-16s drv=%-10s alias=%-5u model=%-10s mode=%-6s enc=%2dbit "
        "KP=%-6d KD=%-5d  %s\n",
        i, j.name.c_str(), j.driver_name.c_str(), j.alias,
        j.model.empty() ? "(default)" : j.model.c_str(),
        j.operation_mode_name.c_str(), j.encoder_bits, j.pvt_kp, j.pvt_kd,
        j.selectable ? "OK" : ("UNAVAILABLE: " + j.unavailable_reason).c_str());
  }

  std::printf("\nEnter joint index(es) to tune (single '3' or comma-separated "
              "'0,2,5'),\n"
              "or 'q' to quit: ");
  std::fflush(stdout);

  std::string line;
  if (!std::getline(std::cin, line)) {
    return {};
  }
  if (line == "q" || line == "Q") {
    return {};
  }

  std::vector<std::size_t> picks;
  std::string token;
  bool bad = false;

  auto flush_token = [&](bool &local_bad) {
    std::size_t a = token.find_first_not_of(" \t\r\n");
    std::size_t b = token.find_last_not_of(" \t\r\n");
    if (a == std::string::npos) {
      token.clear();
      return;
    }
    std::string trimmed = token.substr(a, b - a + 1);
    token.clear();

    try {
      const int idx = std::stoi(trimmed);
      if (idx < 0 || static_cast<std::size_t>(idx) >= joints.size()) {
        std::cerr << "Index '" << trimmed << "' out of range\n";
        local_bad = true;
        return;
      }
      if (!joints[static_cast<std::size_t>(idx)].selectable) {
        std::cerr << "Joint '" << joints[static_cast<std::size_t>(idx)].name
                  << "' is not selectable: "
                  << joints[static_cast<std::size_t>(idx)].unavailable_reason
                  << "\n";
        local_bad = true;
        return;
      }
      for (std::size_t existing : picks) {
        if (existing == static_cast<std::size_t>(idx)) {
          std::cerr << "Duplicate index '" << trimmed << "' in selection\n";
          local_bad = true;
          return;
        }
      }
      picks.push_back(static_cast<std::size_t>(idx));
    } catch (...) {
      std::cerr << "Invalid token: '" << trimmed << "'\n";
      local_bad = true;
    }
  };

  for (char ch : line) {
    if (ch == ',') {
      flush_token(bad);
    } else {
      token.push_back(ch);
    }
  }
  flush_token(bad);

  if (bad || picks.empty()) {
    return {static_cast<std::size_t>(-1)};
  }
  return picks;
}

std::optional<TrajectoryMode> pick_mode(TrajectoryMode previous) {
  std::printf("\n--- Trajectory mode ---\n");
  std::printf("  [s] sin       -- fixed-frequency sinusoid about the captured "
              "mid-point\n");
  std::printf(
      "  [l] chirp-lin -- linear frequency sweep f0->f1->f0 (bode/sweep ID)\n");
  std::printf(
      "  [g] chirp-log -- logarithmic frequency sweep (wide-band ID)\n");
  std::printf(
      "  [t] triangle  -- constant-velocity triangle sweep (friction/range)\n");
  std::printf("  [e] step      -- square-wave step response\n");
  std::printf("  [m] multisine -- summed harmonics, Schroeder phasing "
              "(one-shot FRF)\n");
  std::printf("  [p] spline    -- replay a Catmull-Rom spline through "
              "waypoints you record\n");
  std::printf("                   by backdriving the joint(s) during idle\n");
  std::printf("\nEnter a key (Enter = keep previous '%s'), or 'q' to quit: ",
              mode_to_string(previous));
  std::fflush(stdout);

  std::string line;
  if (!std::getline(std::cin, line)) {
    return std::nullopt;
  }

  std::size_t a = line.find_first_not_of(" \t\r\n");
  std::size_t b = line.find_last_not_of(" \t\r\n");
  if (a == std::string::npos) {
    return previous;
  }

  std::string trimmed = line.substr(a, b - a + 1);
  if (trimmed == "q" || trimmed == "Q") {
    return std::nullopt;
  }
  if (trimmed == "s" || trimmed == "S" || trimmed == "sin") {
    return TrajectoryMode::Sin;
  }
  if (trimmed == "l" || trimmed == "L" || trimmed == "chirp-linear") {
    return TrajectoryMode::ChirpLinear;
  }
  if (trimmed == "g" || trimmed == "G" || trimmed == "chirp-log") {
    return TrajectoryMode::ChirpLog;
  }
  if (trimmed == "t" || trimmed == "T" || trimmed == "triangle") {
    return TrajectoryMode::Triangle;
  }
  if (trimmed == "e" || trimmed == "E" || trimmed == "step") {
    return TrajectoryMode::Step;
  }
  if (trimmed == "m" || trimmed == "M" || trimmed == "multisine") {
    return TrajectoryMode::Multisine;
  }
  if (trimmed == "p" || trimmed == "P" || trimmed == "spline") {
    return TrajectoryMode::Spline;
  }

  std::cerr << "Unrecognised mode '" << trimmed << "', keeping previous.\n";
  return previous;
}

void idle_all(std::vector<JointHandle> &joints) {
  for (auto &j : joints) {
    if (!j.driver) {
      continue;
    }
    j.driver->set_target_position(j.driver->actual_position());
    j.driver->set_target_velocity(0);
    j.driver->idle();
  }
}

} // namespace actuator_test
