#pragma once

#include "actuator_test/types.hpp"

#include <ethercat-primer/core>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace actuator_test
{

enum class DriverKind
{
    MyActuator,
    Novanta,
};

class DriverAdapter
{
public:
    virtual ~DriverAdapter() = default;

    virtual DriverKind kind() const noexcept = 0;
    virtual const char *kind_name() const noexcept = 0;
    virtual int32_t actual_position() const noexcept = 0;
    virtual int32_t actual_velocity() const noexcept = 0;
    virtual uint16_t status() const noexcept = 0;
    virtual bool fault() const noexcept = 0;
    virtual uint16_t error_code() const noexcept = 0;

    virtual bool has_temperature_feedback() const noexcept = 0;
    virtual int16_t motor_temperature() const noexcept = 0;
    virtual int16_t drive_temperature() const noexcept = 0;
    virtual int8_t operation_mode_display() const noexcept = 0;

    virtual void set_target_position(int32_t value) noexcept = 0;
    virtual void set_target_velocity(int32_t value) noexcept = 0;
    virtual void update_operation_mode(int8_t op_mode) noexcept = 0;
    virtual void idle() noexcept = 0;

    virtual uint16_t encoder_bits() const noexcept
    {
        return 0;
    }

    virtual void apply_runtime_gains(int32_t, int32_t) noexcept
    {
    }
};

struct JointHandle
{
    std::string name;
    uint16_t alias = 0;
    std::string driver_name;
    std::string model;
    std::string operation_mode_name;
    int8_t operation_mode_code = ecp::DS402::OP_HOM;
    int encoder_bits = 17;
    int32_t pvt_kp = 200;
    int32_t pvt_kd = 50;
    bool selectable = false;
    std::string unavailable_reason;
    std::shared_ptr<DriverAdapter> driver;
};

class DriverFactory
{
public:
    virtual ~DriverFactory() = default;

    virtual std::string_view driver_name() const noexcept = 0;
    virtual std::optional<JointHandle> create(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg,
                                              const std::string &device_name) const = 0;
};

std::optional<int8_t> ds402_mode_from_name(const std::string &name);

const DriverFactory *find_driver_factory(const std::string &driver_name);

std::string supported_driver_names();

std::vector<JointHandle> enumerate_joints(const ecp::EthercatBus &bus, const ecp::DeviceConfig &cfg);

std::vector<std::size_t> pick_joints(const std::vector<JointHandle> &joints);

std::optional<TrajectoryMode> pick_mode(TrajectoryMode previous);

void idle_all(std::vector<JointHandle> &joints);

} // namespace actuator_test
