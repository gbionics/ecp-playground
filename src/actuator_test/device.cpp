#include "actuator_test/device.hpp"

#include "actuator_test/types.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>
#include <memory>
#include <span>
#include <string_view>

namespace actuator_test
{

  namespace
  {

    struct ModeName
    {
      std::string_view name;
      int8_t code;
    };

    constexpr ModeName kDs402Modes[] = {
        {"OP_PP", ecp::DS402::OP_PP},
        {"OP_PV", ecp::DS402::OP_PV},
        {"OP_TQ", ecp::DS402::OP_TQ},
        {"OP_PVT", ecp::DS402::OP_PVT},
        {"OP_HOM", ecp::DS402::OP_HOM},
        {"OP_CSP", ecp::DS402::OP_CSP},
        {"OP_CSV", ecp::DS402::OP_CSV},
        {"OP_CST", ecp::DS402::OP_CST},
    };

    constexpr ModeName kSynapticonJDXModes[] = {
        {"OP_PP", ecp::SynapticonJDX::OP_PROFILE_POSITION},
        {"OP_PROFILE_POSITION", ecp::SynapticonJDX::OP_PROFILE_POSITION},
        {"OP_PV", ecp::SynapticonJDX::OP_PROFILE_VELOCITY},
        {"OP_PROFILE_VELOCITY", ecp::SynapticonJDX::OP_PROFILE_VELOCITY},
        {"OP_TQ", ecp::SynapticonJDX::OP_PROFILE_TORQUE},
        {"OP_PROFILE_TORQUE", ecp::SynapticonJDX::OP_PROFILE_TORQUE},
        {"OP_CSP", ecp::SynapticonJDX::OP_CSP},
        {"OP_CSV", ecp::SynapticonJDX::OP_CSV},
        {"OP_CST", ecp::SynapticonJDX::OP_CST},
        {"OP_DIAGNOSTICS", ecp::SynapticonJDX::OP_DIAGNOSTICS},
        {"OP_JOINT_TORQUE_CONTROL", ecp::SynapticonJDX::OP_JOINT_TORQUE_CONTROL},
        {"OP_IMP", ecp::SynapticonJDX::OP_IMP},
        {"OP_CURRENT_CONTROL", ecp::SynapticonJDX::OP_CURRENT_CONTROL},
    };

    std::optional<int8_t> mode_from_name(std::span<const ModeName> table,
                                         std::string_view name)
    {
      for (const auto &entry : table)
      {
        if (entry.name == name)
        {
          return entry.code;
        }
      }
      return std::nullopt;
    }

    double finite_or_zero(double value) noexcept
    {
      return std::isfinite(value) ? value : 0.0;
    }

    double torque_constant_for_model(std::string_view model) noexcept
    {
      if (model == "X10-200")
        return 3.3;
      if (model == "X8-120")
        return 2.0;
      if (model == "X6-60")
        return 1.5;
      if (model == "X4-36")
        return 1.2;
      if (model == "X4-10")
        return 0.8;
      return 0.0;
    }

    // Accessors that every DS402-based ethercat-primer driver exposes identically.
    template <typename Driver, DriverKind Kind>
    class Ds402Adapter : public DriverAdapter
    {
    public:
      Ds402Adapter(std::shared_ptr<Driver> driver, uint16_t encoder_bits,
                   double torque_constant_nm_per_a)
          : m_driver(std::move(driver)), m_encoder_bits(encoder_bits),
            m_torque_constant_nm_per_a(torque_constant_nm_per_a) {}

      DriverKind kind() const noexcept final { return Kind; }

      int32_t actual_position() const noexcept final
      {
        return m_driver->actual_position();
      }

      int32_t actual_velocity() const noexcept final
      {
        return m_driver->actual_velocity();
      }

      uint16_t status() const noexcept final { return m_driver->status(); }

      bool fault() const noexcept final { return m_driver->fault(); }

      int8_t operation_mode_display() const noexcept final
      {
        return m_driver->op_mode_display();
      }

      void set_target_position(int32_t value) noexcept final
      {
        m_driver->set_target_position(value);
      }

      void set_target_velocity(int32_t value) noexcept final
      {
        m_driver->set_target_velocity(value);
      }

      uint16_t encoder_bits() const noexcept final { return m_encoder_bits; }

      double torque_constant_nm_per_a() const noexcept override
      {
        return m_torque_constant_nm_per_a;
      }

      double rated_torque_nm() const noexcept override
      {
        return rated_current_a() * torque_constant_nm_per_a();
      }

    protected:
      std::shared_ptr<Driver> m_driver;
      uint16_t m_encoder_bits;
      double m_torque_constant_nm_per_a; ///< Configured Kt; 0 when unknown.
    };

    class MyActuatorAdapter final
        : public Ds402Adapter<ecp::MyActuator::Driver, DriverKind::MyActuator>
    {
    public:
      using Ds402Adapter::Ds402Adapter;

      double actual_current_a() const noexcept override
      {
        return static_cast<double>(m_driver->actual_torque()) / 1000.0 *
               rated_current_a();
      }

      double rated_current_a() const noexcept override
      {
        return static_cast<double>(m_driver->rated_current()) / 1000.0;
      }

      uint16_t error_code() const noexcept override
      {
        return m_driver->error_code();
      }

      bool has_temperature_feedback() const noexcept override { return true; }

      int16_t motor_temperature() const noexcept override
      {
        return m_driver->motor_temperature();
      }

      int16_t drive_temperature() const noexcept override
      {
        return m_driver->drive_temperature();
      }

      void set_target_torque(int16_t value_permille) noexcept override
      {
        m_driver->set_target_torque(value_permille);
      }

      void update_operation_mode(int8_t op_mode) noexcept override
      {
        m_driver->update_operation_mode(op_mode);
      }

      void idle() noexcept override { m_driver->idle(); }

      void apply_runtime_gains(int32_t kp, int32_t kd) noexcept override
      {
        m_driver->set_pvt_kp(kp);
        m_driver->set_pvt_kd(kd);
      }
    };

    class NovantaAdapter final
        : public Ds402Adapter<ecp::Novanta::Driver, DriverKind::Novanta>
    {
    public:
      using Ds402Adapter::Ds402Adapter;

      double actual_current_a() const noexcept override
      {
        return static_cast<double>(m_driver->current_quadrature_value());
      }

      double rated_current_a() const noexcept override
      {
        return static_cast<double>(m_driver->motor_rated_current());
      }

      uint16_t error_code() const noexcept override { return 0; }

      bool has_temperature_feedback() const noexcept override { return false; }

      int16_t motor_temperature() const noexcept override { return -1; }

      int16_t drive_temperature() const noexcept override { return -1; }

      void set_target_torque(int16_t value_permille) noexcept override
      {
        set_target_current_a(static_cast<double>(value_permille) / 1000.0 *
                             rated_current_a());
      }

      // Novanta has no DS402 TargetTorque PDO entry; it commands current
      // directly in amperes via CurrentQuadratureSetPoint.
      void set_target_current_a(double amps) noexcept override
      {
        m_driver->set_current_quadrature_setpoint(
            static_cast<float>(std::isfinite(amps) ? amps : 0.0));
      }

      void update_operation_mode(int8_t op_mode) noexcept override
      {
        m_driver->update_operation_mode(op_mode);
      }

      void idle() noexcept override { m_driver->idle(); }

      bool request_fault_reset() noexcept override
      {
        m_driver->request_fault_reset();
        return true;
      }
    };

    class SynapticonJDXAdapter final
        : public Ds402Adapter<ecp::SynapticonJDX::Driver,
                              DriverKind::SynapticonJDX>
    {
    public:
      using Ds402Adapter::Ds402Adapter;

      // Configured Kt, else rated torque / rated current (0x6076 / 0x6075). Kt is
      // in the drive's torque units (0x6071/0x6077, motor side).
      double torque_constant_nm_per_a() const noexcept override
      {
        if (m_torque_constant_nm_per_a > 0.0)
        {
          return m_torque_constant_nm_per_a;
        }
        const double rated_a = rated_current_a();
        return rated_a > 0.0 ? rated_torque_nm() / rated_a : 0.0;
      }

      // No current feedback in the TxPDO: I = actual torque (0x6077) / Kt.
      double actual_current_a() const noexcept override
      {
        const double kt = torque_constant_nm_per_a();
        return kt > 0.0 ? finite_or_zero(m_driver->actual_torque_nm()) / kt : 0.0;
      }

      // Unit-derived values are NaN until the drive's axis-unit SDOs are read.
      double rated_current_a() const noexcept override
      {
        return finite_or_zero(m_driver->motor_rated_current_a());
      }

      double rated_torque_nm() const noexcept override
      {
        return finite_or_zero(m_driver->motor_rated_torque_nm());
      }

      uint16_t error_code() const noexcept override
      {
        return m_driver->error_code();
      }

      bool has_temperature_feedback() const noexcept override { return true; }

      int16_t motor_temperature() const noexcept override
      {
        return m_driver->motor_temperature_c();
      }

      int16_t drive_temperature() const noexcept override
      {
        const double celsius = m_driver->drive_temperature_c();
        return std::isfinite(celsius) ? static_cast<int16_t>(std::lround(celsius))
                                      : int16_t{-1};
      }

      void set_target_torque(int16_t value_permille) noexcept override
      {
        m_driver->set_target_torque(value_permille);
      }

      // Commanded as Target Torque (0x6071) = Kt * I; falls back to zero torque
      // when Kt is unknown or the request is out of range.
      void set_target_current_a(double amps) noexcept override
      {
        const double kt = torque_constant_nm_per_a();
        if (!(kt > 0.0) || !m_driver->set_target_torque_nm(amps * kt))
        {
          m_driver->set_target_torque(0);
        }
      }

      // Queued and applied by the driver's cyclic on_update(), so it is safe from
      // non-RT threads and is dropped while STO/SBC is engaged.
      void update_operation_mode(int8_t op_mode) noexcept override
      {
        m_driver->request_operation_mode(op_mode);
      }

      // Driver::idle() is the bus-forced path and logs an error; go through the
      // regular request queue instead.
      void idle() noexcept override
      {
        m_driver->request_operation_mode(ecp::SynapticonJDX::OP_IDLE);
      }

      bool request_fault_reset() noexcept override
      {
        m_driver->request_fault_reset();
        return true;
      }
    };

    // Shared JointHandle construction; drivers only customise the hooks below.
    template <typename Driver, typename Adapter>
    class DriverFactoryBase : public DriverFactory
    {
    public:
      std::string_view driver_name() const noexcept final
      {
        return driver_kind_name(kind());
      }

      std::optional<JointHandle>
      create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
             const std::string &device_name) const final
      {
        auto d = bus.get_device<Driver>(device_name);
        if (!d)
        {
          std::cerr << "  skip '" << device_name << "': " << driver_name()
                    << " not present on bus\n";
          return std::nullopt;
        }

        JointHandle jh;
        jh.name = device_name;
        jh.driver_name = std::string(driver_name());
        jh.alias = static_cast<uint16_t>(cfg.get<int>(device_name, "alias", 0));
        jh.model = cfg.get<std::string>(device_name, "model", "");
        jh.pvt_kp = cfg.get<int32_t>(device_name, "PVT_KP", 200);
        jh.pvt_kd = cfg.get<int32_t>(device_name, "PVT_KD", 50);
        jh.encoder_bits =
            cfg.get<int>(device_name, "encoder_bits", default_encoder_bits(*d));
        const auto mode_name = [&](const char *key, std::string_view fallback)
        {
          auto name = cfg.get<std::string>(device_name, key, "");
          return name.empty() ? std::string(fallback) : name;
        };
        jh.operation_mode_name =
            mode_name("operation_mode", default_operation_mode());
        jh.current_mode_name = mode_name(
            "current_operation_mode", default_current_mode().empty()
                                          ? std::string_view(jh.operation_mode_name)
                                          : default_current_mode());

        jh.driver = std::make_shared<Adapter>(
            std::move(d), static_cast<uint16_t>(std::max(jh.encoder_bits, 0)),
            cfg.get<double>(device_name, "torque_constant_nm_per_a",
                            torque_constant_for_model(jh.model)));
        jh.rated_current_a = jh.driver->rated_current_a();
        jh.rated_torque_nm = jh.driver->rated_torque_nm();
        jh.torque_constant_nm_per_a = jh.driver->torque_constant_nm_per_a();

        jh.unavailable_reason = availability(jh);
        jh.selectable = jh.unavailable_reason.empty();
        return jh;
      }

    protected:
      virtual DriverKind kind() const noexcept = 0;
      virtual std::span<const ModeName> mode_names() const noexcept
      {
        return kDs402Modes;
      }
      virtual std::string_view default_operation_mode() const noexcept
      {
        return "OP_CSP";
      }
      // Empty: current tests reuse the operation mode.
      virtual std::string_view default_current_mode() const noexcept { return {}; }
      virtual int default_encoder_bits(const Driver &d) const noexcept
      {
        return static_cast<int>(d.encoder_bits());
      }
      // Driver-specific constraint on a parsed mode; returns a reason or empty.
      virtual std::string mode_restriction(std::string_view /*key*/,
                                           const std::string & /*name*/,
                                           int8_t /*code*/) const
      {
        return {};
      }

    private:
      std::string parse_mode(std::string_view key, const std::string &name,
                             int8_t &code) const
      {
        const auto op = mode_from_name(mode_names(), name);
        if (!op)
        {
          return "unknown " + std::string(key) + "='" + name + "'";
        }
        code = *op;
        return mode_restriction(key, name, code);
      }

      std::string availability(JointHandle &jh) const
      {
        if (auto reason = parse_mode("operation_mode", jh.operation_mode_name,
                                     jh.operation_mode_code);
            !reason.empty())
        {
          return reason;
        }
        if (auto reason = parse_mode("current_operation_mode", jh.current_mode_name,
                                     jh.current_mode_code);
            !reason.empty())
        {
          return reason;
        }
        if (jh.encoder_bits <= 0)
        {
          return "encoder resolution unknown (set encoder_bits)";
        }
        return {};
      }
    };

    class MyActuatorFactory final
        : public DriverFactoryBase<ecp::MyActuator::Driver, MyActuatorAdapter>
    {
    protected:
      DriverKind kind() const noexcept override { return DriverKind::MyActuator; }

      std::string_view default_operation_mode() const noexcept override
      {
        return "OP_PVT";
      }

      std::string mode_restriction(std::string_view key, const std::string &name,
                                   int8_t code) const override
      {
        if (code == ecp::DS402::OP_PVT)
        {
          return {};
        }
        return std::string(key) + "='" + name +
               "' (MT_Device requires OP_PVT for impedance TxPDO watchdog)";
      }
    };

    class NovantaFactory final
        : public DriverFactoryBase<ecp::Novanta::Driver, NovantaAdapter>
    {
    protected:
      DriverKind kind() const noexcept override { return DriverKind::Novanta; }

      int default_encoder_bits(
          const ecp::Novanta::Driver &) const noexcept override
      {
        return 13;
      }
    };

    class SynapticonJDXFactory final
        : public DriverFactoryBase<ecp::SynapticonJDX::Driver,
                                   SynapticonJDXAdapter>
    {
    protected:
      DriverKind kind() const noexcept override
      {
        return DriverKind::SynapticonJDX;
      }

      std::span<const ModeName> mode_names() const noexcept override
      {
        return kSynapticonJDXModes;
      }

      // Firmware OP_CURRENT_CONTROL (-7) did not respond to 0x6071 on hardware;
      // CST takes the Kt-converted torque setpoint directly.
      std::string_view default_current_mode() const noexcept override
      {
        return "OP_CST";
      }

      std::string mode_restriction(std::string_view key, const std::string &name,
                                   int8_t code) const override
      {
        if (code != ecp::SynapticonJDX::OP_DIAGNOSTICS)
        {
          return {};
        }
        return std::string(key) + "='" + name + "' is not a control mode";
      }
    };

    const std::vector<std::unique_ptr<DriverFactory>> &driver_factories()
    {
      static const std::vector<std::unique_ptr<DriverFactory>> factories = []
      {
        std::vector<std::unique_ptr<DriverFactory>> out;
        out.push_back(std::make_unique<MyActuatorFactory>());
        out.push_back(std::make_unique<NovantaFactory>());
        out.push_back(std::make_unique<SynapticonJDXFactory>());
        return out;
      }();
      return factories;
    }

  } // namespace

  const char *driver_kind_name(DriverKind kind) noexcept
  {
    switch (kind)
    {
    case DriverKind::MyActuator:
      return "MT_Device";
    case DriverKind::Novanta:
      return "CapitanDrv";
    case DriverKind::SynapticonJDX:
      return "SynapticonJDX";
    }
    return "unknown";
  }

  std::optional<int8_t> ds402_mode_from_name(const std::string &name)
  {
    return mode_from_name(kDs402Modes, name);
  }

  const DriverFactory *find_driver_factory(const std::string &driver_name)
  {
    for (const auto &factory : driver_factories())
    {
      if (factory->driver_name() == driver_name)
      {
        return factory.get();
      }
    }
    return nullptr;
  }

  std::string supported_driver_names()
  {
    std::string out;
    for (const auto &factory : driver_factories())
    {
      if (!out.empty())
      {
        out += ", ";
      }
      out += factory->driver_name();
    }
    return out;
  }

  std::vector<JointHandle> enumerate_joints(const ecp::EthercatBus &bus,
                                            const ecp::DeviceConfig &cfg)
  {
    std::vector<JointHandle> out;
    for (const auto &name : cfg.device_names())
    {
      const std::string driver_name = cfg.get<std::string>(name, "driver", "");
      const DriverFactory *factory = find_driver_factory(driver_name);
      if (factory == nullptr)
      {
        continue;
      }
      if (auto jh = factory->create(bus, cfg, name))
      {
        out.push_back(std::move(*jh));
      }
    }

    return out;
  }

  std::vector<std::size_t> pick_joints(const std::vector<JointHandle> &joints)
  {
    std::printf("\n--- Joint picker ---\n");
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
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
    if (!std::getline(std::cin, line))
    {
      return {};
    }
    if (line == "q" || line == "Q")
    {
      return {};
    }

    std::vector<std::size_t> picks;
    std::string token;
    bool bad = false;

    auto flush_token = [&](bool &local_bad)
    {
      std::size_t a = token.find_first_not_of(" \t\r\n");
      std::size_t b = token.find_last_not_of(" \t\r\n");
      if (a == std::string::npos)
      {
        token.clear();
        return;
      }
      std::string trimmed = token.substr(a, b - a + 1);
      token.clear();

      try
      {
        const int idx = std::stoi(trimmed);
        if (idx < 0 || static_cast<std::size_t>(idx) >= joints.size())
        {
          std::cerr << "Index '" << trimmed << "' out of range\n";
          local_bad = true;
          return;
        }
        if (!joints[static_cast<std::size_t>(idx)].selectable)
        {
          std::cerr << "Joint '" << joints[static_cast<std::size_t>(idx)].name
                    << "' is not selectable: "
                    << joints[static_cast<std::size_t>(idx)].unavailable_reason
                    << "\n";
          local_bad = true;
          return;
        }
        for (std::size_t existing : picks)
        {
          if (existing == static_cast<std::size_t>(idx))
          {
            std::cerr << "Duplicate index '" << trimmed << "' in selection\n";
            local_bad = true;
            return;
          }
        }
        picks.push_back(static_cast<std::size_t>(idx));
      }
      catch (...)
      {
        std::cerr << "Invalid token: '" << trimmed << "'\n";
        local_bad = true;
      }
    };

    for (char ch : line)
    {
      if (ch == ',')
      {
        flush_token(bad);
      }
      else
      {
        token.push_back(ch);
      }
    }
    flush_token(bad);

    if (bad || picks.empty())
    {
      return {static_cast<std::size_t>(-1)};
    }
    return picks;
  }

  std::optional<TrajectoryMode> pick_mode(TrajectoryMode previous)
  {
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
    if (!std::getline(std::cin, line))
    {
      return std::nullopt;
    }

    std::size_t a = line.find_first_not_of(" \t\r\n");
    std::size_t b = line.find_last_not_of(" \t\r\n");
    if (a == std::string::npos)
    {
      return previous;
    }

    std::string trimmed = line.substr(a, b - a + 1);
    if (trimmed == "q" || trimmed == "Q")
    {
      return std::nullopt;
    }
    if (trimmed == "s" || trimmed == "S" || trimmed == "sin")
    {
      return TrajectoryMode::Sin;
    }
    if (trimmed == "l" || trimmed == "L" || trimmed == "chirp-linear")
    {
      return TrajectoryMode::ChirpLinear;
    }
    if (trimmed == "g" || trimmed == "G" || trimmed == "chirp-log")
    {
      return TrajectoryMode::ChirpLog;
    }
    if (trimmed == "t" || trimmed == "T" || trimmed == "triangle")
    {
      return TrajectoryMode::Triangle;
    }
    if (trimmed == "e" || trimmed == "E" || trimmed == "step")
    {
      return TrajectoryMode::Step;
    }
    if (trimmed == "m" || trimmed == "M" || trimmed == "multisine")
    {
      return TrajectoryMode::Multisine;
    }
    if (trimmed == "p" || trimmed == "P" || trimmed == "spline")
    {
      return TrajectoryMode::Spline;
    }

    std::cerr << "Unrecognised mode '" << trimmed << "', keeping previous.\n";
    return previous;
  }

  void idle_all(std::vector<JointHandle> &joints)
  {
    for (auto &j : joints)
    {
      if (!j.driver)
      {
        continue;
      }
      j.driver->set_target_position(j.driver->actual_position());
      j.driver->set_target_velocity(0);
      j.driver->idle();
    }
  }

} // namespace actuator_test
