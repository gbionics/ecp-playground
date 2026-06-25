// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "core/controller_worker.hpp"

#include "actuator_test/device.hpp"
#include "actuator_test/logging.hpp"
#include "actuator_test/math.hpp"
#include "actuator_test/runtime.hpp"
#include "actuator_test/safety.hpp"
#include "actuator_test/session.hpp"
#include "actuator_test/trajectory.hpp"

#include <ethercat-primer/core>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace actuator_test::gui
{

const char *to_string(ControllerState state) noexcept
{
    switch (state)
    {
    case ControllerState::Disconnected:
        return "disconnected";
    case ControllerState::Connected:
        return "connected";
    case ControllerState::Jogging:
        return "jogging";
    case ControllerState::Capturing:
        return "capturing";
    case ControllerState::Running:
        return "running";
    case ControllerState::Faulted:
        return "faulted";
    }
    return "?";
}

const char *to_string(CiA402State state) noexcept
{
    switch (state)
    {
    case CiA402State::NotReadyToSwitchOn:
        return "Not Ready to Switch On";
    case CiA402State::SwitchedOnDisabled:
        return "Switched On Disabled";
    case CiA402State::ReadyToSwitchOn:
        return "Ready to Switch On";
    case CiA402State::SwitchedOn:
        return "Switched On";
    case CiA402State::OperationEnabled:
        return "Operation Enabled";
    case CiA402State::QuickStop:
        return "Quick Stop";
    case CiA402State::FaultReactionActive:
        return "Fault Reaction Active";
    case CiA402State::Fault:
        return "Fault";
    case CiA402State::Unknown:
        return "Unknown";
    }
    return "?";
}

namespace
{

/// Map CiA-402 device status word to state machine enum.
/// DS402 status word bits: 0-3 (state), 4 (voltage enabled), 5 (quick stop), 6 (enable),
/// 7 (fault), 14 (warning), 15 (manufacturer)
CiA402State map_status_to_cia402_state(uint16_t status_word, bool fault) noexcept
{
    if (fault)
        return CiA402State::Fault;

    // Extract state bits (bits 0-3)
    const uint16_t state_bits = status_word & 0x000F;

    // DS402 state machine (bits 0-3, some use bit 7 for fault)
    // 0x0X = Not Ready to Switch On
    // 0x1X = Switch on disabled
    // 0x2X = Ready to Switch On (bit 5=1) or Switched On Disabled (bit 5=0)
    // 0x3X = Switched On (transition) or Operation Enabled
    // 0x7X = Quick Stop Active

    if (state_bits >= 0x08)
    {
        return CiA402State::Fault;
    }

    const bool voltage_enabled = (status_word & 0x0010) != 0;
    const bool quick_stop = (status_word & 0x0020) != 0;
    const bool enabled = (status_word & 0x0040) != 0;

    // Standard DS402 state transitions
    if ((state_bits & 0x06) == 0x00)
    {
        return CiA402State::NotReadyToSwitchOn;
    }
    else if ((state_bits & 0x06) == 0x02)
    {
        if (voltage_enabled)
            return CiA402State::ReadyToSwitchOn;
        else
            return CiA402State::SwitchedOnDisabled;
    }
    else if ((state_bits & 0x06) == 0x04)
    {
        if (!quick_stop)
            return CiA402State::QuickStop;
        else if (enabled)
            return CiA402State::OperationEnabled;
        else
            return CiA402State::SwitchedOn;
    }
    else if ((state_bits & 0x06) == 0x06)
    {
        return CiA402State::FaultReactionActive;
    }

    return CiA402State::Unknown;
}

} // namespace

/// Per-joint control activity inside the real-time loop.
enum class Activity
{
    Idle,       ///< Drive idled / backdrivable.
    Jog,        ///< Velocity-integrating manual move.
    GoTo,       ///< Min-jerk move to an absolute target, then hold.
    Capture,    ///< Idled while tracking the travelled min/max.
    Trajectory, ///< Playing a generated trajectory.
};

/// Decimate a raw recorded position trail into spline waypoints, mirroring the
/// CLI logic: drop near-stationary samples, always keep the endpoints, then
/// thin uniformly to the configured maximum. Returns an empty vector if the
/// motion is too short to form a usable spline.
std::vector<int32_t> decimate_trail(const std::vector<int32_t> &trail, int encoder_bits, const RuntimeProfile &profile)
{
    if (trail.size() < 2)
    {
        return {};
    }

    std::vector<std::size_t> kept;
    kept.reserve(trail.size());
    kept.push_back(0);
    for (std::size_t i = 1; i + 1 < trail.size(); ++i)
    {
        const double d_deg = std::fabs(counts2deg(trail[i] - trail[kept.back()], encoder_bits));
        if (d_deg >= profile.waypoint_decimation_deg)
        {
            kept.push_back(i);
        }
    }
    // Always terminate on the final sample.
    const double tail_deg = std::fabs(counts2deg(trail.back() - trail[kept.back()], encoder_bits));
    if (tail_deg >= profile.waypoint_decimation_deg)
    {
        kept.push_back(trail.size() - 1);
    }
    else
    {
        kept.back() = trail.size() - 1;
    }

    if (kept.size() > profile.max_spline_waypoints)
    {
        std::vector<std::size_t> thinned;
        thinned.reserve(profile.max_spline_waypoints);
        const double step =
            static_cast<double>(kept.size() - 1) / static_cast<double>(profile.max_spline_waypoints - 1);
        for (std::size_t k = 0; k < profile.max_spline_waypoints; ++k)
        {
            const std::size_t idx = static_cast<std::size_t>(std::round(k * step));
            const std::size_t src = std::min(kept.size() - 1, idx);
            if (thinned.empty() || thinned.back() != kept[src])
            {
                thinned.push_back(kept[src]);
            }
        }
        kept.swap(thinned);
    }

    if (kept.size() < profile.min_spline_waypoints)
    {
        return {};
    }

    std::vector<int32_t> waypoints;
    waypoints.reserve(kept.size());
    for (std::size_t k : kept)
    {
        waypoints.push_back(trail[k]);
    }
    return waypoints;
}

/// All EtherCAT-owning, control-thread-only state lives here.
struct ControllerWorker::Impl
{
    struct JointCtl
    {
        Activity activity = Activity::Idle;
        bool engaged = false;

        int32_t min_counts = 0;
        int32_t max_counts = 0;
        bool limits_set = false; ///< True once a valid (min<max) envelope exists.

        // Jog / GoTo
        double cmd_counts = 0.0; ///< Integrated command position.
        double jog_vel_deg_s = 0.0;
        double goto_start = 0.0;
        double goto_target = 0.0;
        double goto_T = 0.0;
        double goto_t = 0.0;

        // Trajectory
        int phase = 0; ///< ControlPhase
        double phase_t = 0.0;
        double start_counts = 0.0;
        double approach_T = 0.0;
        std::unique_ptr<Trajectory> traj;
        std::unique_ptr<LowPass> lpf;
        SafetyState safety;
        std::unique_ptr<JointCsvLogger> logger;

        // Spline recording (populated while backdriving in Capture).
        std::vector<int32_t> trail_counts;
        double trail_t = 0.0; ///< Time accumulator for the record decimator.

        // Telemetry tracking for acceleration calculation
        double prev_velocity_deg_s = 0.0;
        double soft_min_counts = 0.0;
        double soft_max_counts = 0.0;
    };

    ecp::EthercatBus bus;
    std::shared_ptr<ecp::SubDeviceMap> subdevices;
    std::shared_ptr<ecp::DeviceConfig> cfg;
    std::vector<JointHandle> joints;
    std::vector<JointCtl> ctl;

    bool bus_up = false;
    bool rt_initialised = false;

    std::chrono::steady_clock::time_point t0;
    int64_t tick = 0;
    std::string log_dir;
};

// ---------------------------------------------------------------------------
//  Construction / lifecycle
// ---------------------------------------------------------------------------

ControllerWorker::ControllerWorker(RuntimeProfile profile)
    : m_impl(std::make_unique<Impl>()), m_profile(std::move(profile))
{
}

ControllerWorker::~ControllerWorker()
{
    post(ShutdownCommand{});
    if (m_thread.joinable())
    {
        m_thread.join();
    }
}

void ControllerWorker::start()
{
    m_thread = std::thread([this] { run(); });
}

// ---------------------------------------------------------------------------
//  Cross-thread channels
// ---------------------------------------------------------------------------

void ControllerWorker::post(Command cmd)
{
    {
        std::lock_guard<std::mutex> lk(m_cmd_mutex);
        m_commands.push_back(std::move(cmd));
    }
    m_cmd_cv.notify_one();
}

bool ControllerWorker::popCommand(Command &out)
{
    std::lock_guard<std::mutex> lk(m_cmd_mutex);
    if (m_commands.empty())
    {
        return false;
    }
    out = std::move(m_commands.front());
    m_commands.pop_front();
    return true;
}

void ControllerWorker::waitForCommand()
{
    std::unique_lock<std::mutex> lk(m_cmd_mutex);
    m_cmd_cv.wait(lk, [this] { return !m_commands.empty() || m_shutdown.load(); });
}

TelemetryFrame ControllerWorker::snapshot() const
{
    std::lock_guard<std::mutex> lk(m_snap_mutex);
    return m_snapshot;
}

void ControllerWorker::publish(TelemetryFrame frame)
{
    std::lock_guard<std::mutex> lk(m_snap_mutex);
    m_snapshot = std::move(frame);
}

std::vector<WorkerEvent> ControllerWorker::drainEvents()
{
    std::lock_guard<std::mutex> lk(m_event_mutex);
    std::vector<WorkerEvent> out;
    out.swap(m_events);
    return out;
}

void ControllerWorker::pushEvent(WorkerEvent ev)
{
    std::lock_guard<std::mutex> lk(m_event_mutex);
    m_events.push_back(std::move(ev));
}

void ControllerWorker::log(std::string msg)
{
    pushEvent({WorkerEvent::Kind::Log, std::move(msg), m_state.load()});
}

void ControllerWorker::error(std::string msg)
{
    pushEvent({WorkerEvent::Kind::Error, std::move(msg), m_state.load()});
}

void ControllerWorker::setState(ControllerState s, std::string reason)
{
    m_state.store(s);
    pushEvent({WorkerEvent::Kind::StateChanged, std::move(reason), s});
}

std::vector<JointInfo> ControllerWorker::joints() const
{
    std::lock_guard<std::mutex> lk(m_joints_mutex);
    return m_joints;
}

ControllerState ControllerWorker::state() const noexcept
{
    return m_state.load();
}

// ---------------------------------------------------------------------------
//  Worker thread
// ---------------------------------------------------------------------------

void ControllerWorker::engageJoint(std::size_t i)
{
    auto &jh = m_impl->joints[i];
    auto &c = m_impl->ctl[i];
    auto &d = *jh.driver;
    d.set_target_velocity(0);
    d.set_target_position(d.actual_position());
    d.apply_runtime_gains(jh.pvt_kp, jh.pvt_kd);
    d.update_operation_mode(jh.operation_mode_code);
    c.cmd_counts = static_cast<double>(d.actual_position());
    c.engaged = true;
}

void ControllerWorker::idleJoint(std::size_t i)
{
    auto &jh = m_impl->joints[i];
    auto &c = m_impl->ctl[i];
    if (!jh.driver)
    {
        return;
    }
    auto &d = *jh.driver;
    d.set_target_position(d.actual_position());
    d.set_target_velocity(0);
    d.idle();
    c.engaged = false;
    c.activity = Activity::Idle;
    c.jog_vel_deg_s = 0.0;
    if (c.logger && c.logger->is_open())
    {
        c.logger->close();
    }
    c.logger.reset();
    c.traj.reset();
    c.lpf.reset();
}

void ControllerWorker::run()
{
    auto &impl = *m_impl;

    while (!m_shutdown.load())
    {
        if (m_state.load() == ControllerState::Disconnected)
        {
            waitForCommand();
            Command cmd;
            while (popCommand(cmd))
            {
                std::visit([&](auto &&c) { this->handleCommand(c); }, cmd);
                if (m_shutdown.load())
                {
                    break;
                }
            }
            continue;
        }

        // --- one 1 kHz control tick ---------------------------------------
        Command cmd;
        while (popCommand(cmd))
        {
            std::visit([&](auto &&c) { this->handleCommand(c); }, cmd);
        }
        if (m_shutdown.load())
        {
            break;
        }
        if (m_state.load() == ControllerState::Disconnected)
        {
            continue;
        }

        controlTick();

        ++impl.tick;
        std::this_thread::sleep_until(
            impl.t0 + std::chrono::microseconds(static_cast<int64_t>(impl.tick * m_profile.loop_period_s() * 1e6)));
    }

    // Shutdown: idle everything.
    if (impl.bus_up)
    {
        for (std::size_t i = 0; i < impl.joints.size(); ++i)
        {
            idleJoint(i);
        }
    }
}

// ---------------------------------------------------------------------------
//  Command handlers
// ---------------------------------------------------------------------------

void ControllerWorker::handleCommand(const ConnectCommand &c)
{
    auto &impl = *m_impl;

    if (impl.bus_up)
    {
        // Already on the bus: just re-idle and report connected.
        for (std::size_t i = 0; i < impl.joints.size(); ++i)
        {
            idleJoint(i);
        }
        impl.t0 = std::chrono::steady_clock::now();
        impl.tick = 0;
        setState(ControllerState::Connected, "reconnected");
        return;
    }

    auto opt_cfg = ecp::DeviceConfig::from_file(c.config_path);
    if (!opt_cfg)
    {
        error("failed to parse config: " + c.config_path);
        return;
    }
    impl.cfg = std::make_shared<ecp::DeviceConfig>(std::move(*opt_cfg));

    if (!impl.rt_initialised)
    {
        if (ecp::rt_app_t::instance().init() != 0)
        {
            error("rt_app init failed");
            return;
        }
        try_enable_realtime_scheduler(m_profile);
        impl.rt_initialised = true;
    }

    const std::string ifname = ecp::get_ethercat_interface();
    if (ifname.empty())
    {
        error("no EtherCAT interface found");
        return;
    }

    impl.subdevices = std::make_shared<ecp::SubDeviceMap>();
    if (impl.bus.startup(ifname, impl.subdevices, impl.cfg) < 0)
    {
        error("bus startup failed on '" + ifname + "'");
        return;
    }

    impl.joints = enumerate_joints(impl.bus, *impl.cfg);
    if (impl.joints.empty())
    {
        error("no supported joints matched. Supported drivers: " + supported_driver_names());
        return;
    }

    impl.ctl.clear();
    impl.ctl.resize(impl.joints.size());
    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        auto &jh = impl.joints[i];
        auto &ct = impl.ctl[i];
        const int32_t now = jh.driver ? jh.driver->actual_position() : 0;
        ct.min_counts = now;
        ct.max_counts = now;
        ct.soft_min_counts = now;
        ct.soft_max_counts = now;
        ct.cmd_counts = static_cast<double>(now);
        ct.limits_set = false;
        ct.trail_counts.clear();
        ct.trail_t = 0.0;
        if (jh.driver)
        {
            idleJoint(i);              // leave the drive backdrivable
            ct.activity = Activity::Capture; // auto-track the range as it is moved by hand
        }
    }

    impl.bus_up = true;
    impl.t0 = std::chrono::steady_clock::now();
    impl.tick = 0;

    publishJointInfo();
    log("connected: " + std::to_string(impl.joints.size()) + " joint(s) on '" + ifname + "'");
    log("backdrive each joint through its range \u2014 limits update live; jogging unlocks once a range is captured");
    setState(ControllerState::Capturing, "connected");
}

void ControllerWorker::handleCommand(const DisconnectCommand &)
{
    auto &impl = *m_impl;
    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        idleJoint(i);
    }
    setState(ControllerState::Disconnected, "disconnected");
    log("all drives idled; bus left up for reconnect");
}

void ControllerWorker::handleCommand(const JogCommand &c)
{
    auto &impl = *m_impl;
    if (c.joint >= impl.joints.size())
    {
        return;
    }
    auto &ct = impl.ctl[c.joint];

    if (ct.activity == Activity::Trajectory)
    {
        return; // Ignore jog during a trajectory run.
    }

    if (std::fabs(c.velocity_deg_s) < 1e-9)
    {
        // Stop jogging but hold position (stay engaged).
        ct.jog_vel_deg_s = 0.0;
        if (ct.activity == Activity::Jog)
        {
            ct.activity = Activity::GoTo;
            ct.goto_start = ct.cmd_counts;
            ct.goto_target = ct.cmd_counts;
            ct.goto_T = 0.0;
            ct.goto_t = 0.0;
        }
        recomputeState();
        return;
    }

    // Powered motion is only allowed once a valid limit envelope has been captured.
    if (!ct.limits_set)
    {
        log("jog blocked on '" + impl.joints[c.joint].name +
            "': backdrive the joint through its range to set limits first");
        return;
    }

    if (!ct.engaged)
    {
        engageJoint(c.joint); // leaves backdrive/capture mode
    }
    ct.activity = Activity::Jog;
    ct.jog_vel_deg_s = c.velocity_deg_s;
    recomputeState();
}

void ControllerWorker::handleCommand(const GoToCommand &c)
{
    auto &impl = *m_impl;
    if (c.joint >= impl.joints.size())
    {
        return;
    }
    auto &jh = impl.joints[c.joint];
    auto &ct = impl.ctl[c.joint];
    if (ct.activity == Activity::Trajectory)
    {
        return;
    }

    // Powered motion is only allowed once a valid limit envelope has been captured.
    if (!ct.limits_set)
    {
        log("move blocked on '" + jh.name + "': backdrive the joint to set limits first");
        return;
    }

    if (!ct.engaged)
    {
        engageJoint(c.joint); // leaves backdrive/capture mode
    }

    const int bits = jh.driver->encoder_bits();
    double target = static_cast<double>(deg2counts(c.target_deg, bits));
    const bool soft_valid = (ct.soft_max_counts > ct.soft_min_counts);
    const double lim_min = soft_valid ? ct.soft_min_counts : static_cast<double>(ct.min_counts);
    const double lim_max = soft_valid ? ct.soft_max_counts : static_cast<double>(ct.max_counts);
    target = std::clamp(target, lim_min, lim_max);

    const double start = ct.cmd_counts;
    const double speed = (c.speed_deg_s > 0.0) ? c.speed_deg_s : m_profile.max_approach_speed_deg_s;
    const double dist_deg = std::fabs(counts2deg(static_cast<int32_t>(target - start), bits));

    ct.activity = Activity::GoTo;
    ct.goto_start = start;
    ct.goto_target = target;
    ct.goto_T = std::max(m_profile.approach_seconds, dist_deg / speed);
    ct.goto_t = 0.0;
    recomputeState();
}

void ControllerWorker::handleCommand(const CaptureLimitsCommand &c)
{
    auto &impl = *m_impl;
    if (c.joints.empty())
    {
        error("no joints selected for capture");
        return;
    }
    for (std::size_t j : c.joints)
    {
        if (j >= impl.joints.size())
        {
            continue;
        }
        auto &ct = impl.ctl[j];
        if (c.start)
        {
            idleJoint(j); // backdrivable
            const int32_t now = impl.joints[j].driver->actual_position();
            ct.min_counts = now;
            ct.max_counts = now;
            ct.soft_min_counts = now;
            ct.soft_max_counts = now;
            ct.limits_set = false;
            ct.trail_counts.clear();
            ct.trail_t = 0.0;
            ct.activity = Activity::Capture;
        }
        else if (ct.activity == Activity::Capture)
        {
            ct.activity = Activity::Idle;
            ct.limits_set = (ct.max_counts > ct.min_counts);
            ct.soft_min_counts = ct.min_counts;
            ct.soft_max_counts = ct.max_counts;
            publishJointInfo();
        }
    }
    recomputeState();
}

void ControllerWorker::handleCommand(const SetLimitsCommand &c)
{
    auto &impl = *m_impl;
    if (c.joint >= impl.joints.size())
    {
        return;
    }
    auto &jh = impl.joints[c.joint];
    auto &ct = impl.ctl[c.joint];
    const int bits = jh.driver->encoder_bits();
    int32_t lo = deg2counts(std::min(c.min_deg, c.max_deg), bits);
    int32_t hi = deg2counts(std::max(c.min_deg, c.max_deg), bits);
    ct.min_counts = lo;
    ct.max_counts = hi;
    ct.soft_min_counts = lo;
    ct.soft_max_counts = hi;
    ct.limits_set = (hi > lo);
    publishJointInfo();
    log("limits set on '" + jh.name + "'");
}

void ControllerWorker::handleCommand(const ResetLimitsCommand &c)
{
    auto &impl = *m_impl;
    if (c.joint >= impl.joints.size())
    {
        return;
    }
    auto &jh = impl.joints[c.joint];
    auto &ct = impl.ctl[c.joint];
    if (!jh.driver)
    {
        return;
    }

    const int32_t now = jh.driver->actual_position();
    ct.min_counts = now;
    ct.max_counts = now;
    ct.soft_min_counts = now;
    ct.soft_max_counts = now;
    ct.limits_set = false;
    ct.trail_counts.clear();
    ct.trail_t = 0.0;
    publishJointInfo();
    log("limits reset on '" + jh.name + "' (learning resumes while idle)");
}

void ControllerWorker::handleCommand(const StartTrajectoryCommand &c)
{
    auto &impl = *m_impl;
    if (c.joints.empty())
    {
        error("no joints selected for trajectory");
        return;
    }

    impl.log_dir.clear();
    if (c.enable_logging)
    {
        impl.log_dir = make_run_log_dir(m_profile);
    }

    bool any = false;
    for (std::size_t j : c.joints)
    {
        if (j >= impl.joints.size())
        {
            continue;
        }
        auto &jh = impl.joints[j];
        auto &ct = impl.ctl[j];
        if (!jh.driver)
        {
            continue;
        }

        const int bits = jh.driver->encoder_bits();
        if (mode_requires_recording(c.mode))
        {
            std::vector<int32_t> waypoints = decimate_trail(ct.trail_counts, bits, m_profile);
            if (waypoints.size() < m_profile.min_spline_waypoints)
            {
                error("'" + jh.name + "': not enough recorded motion for a spline (need >= " +
                      std::to_string(m_profile.min_spline_waypoints) +
                      " waypoints); backdrive the joint through the desired path first");
                continue;
            }
            std::vector<double> knot_times(waypoints.size(), 0.0);
            for (std::size_t k = 1; k < waypoints.size(); ++k)
            {
                const double d_deg = std::fabs(counts2deg(waypoints[k] - waypoints[k - 1], bits));
                const double seg =
                    std::max(m_profile.min_segment_seconds, d_deg / m_profile.max_approach_speed_deg_s);
                knot_times[k] = knot_times[k - 1] + seg;
            }
            const std::size_t n_wp = waypoints.size();
            ct.traj = std::make_unique<SplineTrajectory>(std::move(knot_times), std::move(waypoints), bits);
            log("'" + jh.name + "': spline with " + std::to_string(n_wp) + " waypoint(s) from " +
                std::to_string(ct.trail_counts.size()) + " recorded samples");
        }
        else
        {
            const double range_deg = counts2deg(ct.max_counts - ct.min_counts, bits);
            if (range_deg < m_profile.min_range_deg)
            {
                error("'" + jh.name + "' range " + std::to_string(range_deg) +
                      " deg below floor; capture limits first");
                continue;
            }

            const double centre = static_cast<double>((ct.min_counts + ct.max_counts) / 2);
            const double amp =
                m_profile.traj_safety_factor * static_cast<double>((ct.max_counts - ct.min_counts) / 2);
            ct.traj = make_parametric_trajectory(c.mode, centre, amp, m_profile, bits);
        }

        engageJoint(j);
        ct.start_counts = static_cast<double>(jh.driver->actual_position());
        const double start_deg = counts2deg(static_cast<int32_t>(ct.start_counts), bits);
        const double target_deg = counts2deg(static_cast<int32_t>(ct.traj->approach_target()), bits);
        const double dist_deg = std::fabs(target_deg - start_deg);
        ct.approach_T = std::max(m_profile.approach_seconds, dist_deg / m_profile.max_approach_speed_deg_s);
        ct.phase = 0; // Hold
        ct.phase_t = 0.0;
        ct.safety = SafetyState{};
        ct.lpf = std::make_unique<LowPass>(m_profile.lpf_cutoff_hz, m_profile.loop_period_s(), ct.start_counts);
        ct.cmd_counts = ct.start_counts;
        ct.activity = Activity::Trajectory;

        if (!impl.log_dir.empty())
        {
            // A lightweight JointPlan is needed only for the CSV header.
            JointPlan plan;
            plan.jh = &jh;
            plan.min_counts = ct.min_counts;
            plan.max_counts = ct.max_counts;
            const std::string path = impl.log_dir + "/" + jh.name + ".csv";
            ct.logger = std::make_unique<JointCsvLogger>();
            if (ct.logger->open(path, m_profile, jh, plan, *ct.traj))
            {
                log("logging '" + jh.name + "' -> " + path);
            }
            else
            {
                ct.logger.reset();
                error("cannot open log for '" + jh.name + "'");
            }
        }
        any = true;
    }

    if (any)
    {
        log("trajectory started (mode=" + std::string(mode_to_string(c.mode)) + ")");
        recomputeState();
    }
}

void ControllerWorker::handleCommand(const StopCommand &)
{
    auto &impl = *m_impl;
    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        idleJoint(i);
    }
    log("stopped; all joints idled");
    recomputeState();
}

void ControllerWorker::handleCommand(const PauseCommand &)
{
    auto &impl = *m_impl;
    bool any = false;
    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        auto &jh = impl.joints[i];
        auto &ct = impl.ctl[i];
        if (!jh.driver || ct.activity != Activity::Trajectory)
        {
            continue;
        }

        // Hold the current measured position while keeping the drive engaged.
        const double hold = static_cast<double>(jh.driver->actual_position());
        ct.cmd_counts = hold;
        ct.goto_start = hold;
        ct.goto_target = hold;
        ct.goto_T = 0.0;
        ct.goto_t = 0.0;
        ct.activity = Activity::GoTo;
        any = true;
    }

    if (any)
    {
        log("trajectory paused; holding current positions");
        recomputeState();
    }
}

void ControllerWorker::handleCommand(const ShutdownCommand &)
{
    m_shutdown.store(true);
    m_cmd_cv.notify_all();
}

// ---------------------------------------------------------------------------
//  Control tick + telemetry
// ---------------------------------------------------------------------------

void ControllerWorker::controlTick()
{
    auto &impl = *m_impl;
    const double dt = m_profile.loop_period_s();

    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        auto &jh = impl.joints[i];
        auto &ct = impl.ctl[i];
        if (!jh.driver)
        {
            continue;
        }
        auto &d = *jh.driver;
        const int bits = d.encoder_bits();

        switch (ct.activity)
        {
        case Activity::Idle:
        {
            // In idle/backdrivable mode, continuously learn the travelled envelope.
            const int32_t a = d.actual_position();
            ct.min_counts = std::min(ct.min_counts, a);
            ct.max_counts = std::max(ct.max_counts, a);
            ct.limits_set = (ct.max_counts > ct.min_counts);
            break;
        }

        case Activity::Capture:
        {
            const int32_t a = d.actual_position();
            ct.min_counts = std::min(ct.min_counts, a);
            ct.max_counts = std::max(ct.max_counts, a);
            ct.cmd_counts = static_cast<double>(a); // keep command at the hand-held position
            ct.limits_set = (ct.max_counts > ct.min_counts);
            // Record a position trail for spline training.
            ct.trail_t += dt;
            if (ct.trail_counts.empty() || ct.trail_t >= m_profile.waypoint_record_period_s())
            {
                ct.trail_t = 0.0;
                ct.trail_counts.push_back(a);
            }
            break;
        }

        case Activity::Jog:
        {
            ct.cmd_counts += static_cast<double>(deg2counts(ct.jog_vel_deg_s * dt, bits));
            if (ct.limits_set)
            {
                const bool soft_valid = (ct.soft_max_counts > ct.soft_min_counts);
                const double lim_min = soft_valid ? ct.soft_min_counts : static_cast<double>(ct.min_counts);
                const double lim_max = soft_valid ? ct.soft_max_counts : static_cast<double>(ct.max_counts);
                ct.cmd_counts = std::clamp(ct.cmd_counts, lim_min, lim_max);
            }
            d.apply_runtime_gains(jh.pvt_kp, jh.pvt_kd);
            d.set_target_position(static_cast<int32_t>(ct.cmd_counts));
            d.set_target_velocity(0);
            break;
        }

        case Activity::GoTo:
        {
            ct.cmd_counts = min_jerk(ct.goto_t, ct.goto_T, ct.goto_start, ct.goto_target);
            ct.goto_t += dt;
            d.apply_runtime_gains(jh.pvt_kp, jh.pvt_kd);
            d.set_target_position(static_cast<int32_t>(ct.cmd_counts));
            d.set_target_velocity(0);
            break;
        }

        case Activity::Trajectory:
        {
            std::string reason;
            if (safety_violated(d, m_profile, jh.name, ct.safety, reason))
            {
                error("safety abort on '" + jh.name + "': " + reason);
                idleJoint(i);
                setState(ControllerState::Faulted, reason);
                break;
            }

            double ref_counts = 0.0;
            if (ct.phase == 0) // Hold
            {
                ref_counts = ct.start_counts;
                if (ct.phase_t >= m_profile.pre_ramp_hold_seconds)
                {
                    ct.phase = 1;
                    ct.phase_t = 0.0;
                }
            }
            else if (ct.phase == 1) // Approach
            {
                ref_counts = min_jerk(ct.phase_t, ct.approach_T, ct.start_counts, ct.traj->approach_target());
                if (ct.phase_t >= ct.approach_T)
                {
                    ct.phase = 2;
                    ct.phase_t = 0.0;
                }
            }
            else // Trajectory
            {
                ref_counts = ct.traj->sample(ct.phase_t);
            }

            double filtered = ct.lpf->step(ref_counts);
            const bool soft_valid = (ct.soft_max_counts > ct.soft_min_counts);
            const double lim_min = soft_valid ? ct.soft_min_counts : static_cast<double>(ct.min_counts);
            const double lim_max = soft_valid ? ct.soft_max_counts : static_cast<double>(ct.max_counts);
            filtered = std::clamp(filtered, lim_min, lim_max);
            ct.cmd_counts = filtered;

            const int32_t actual_counts = d.actual_position();
            const int16_t motor_t = d.has_temperature_feedback() ? d.motor_temperature() : -1;
            const int16_t drive_t = d.has_temperature_feedback() ? d.drive_temperature() : -1;
            const uint16_t err = d.error_code();

            d.apply_runtime_gains(jh.pvt_kp, jh.pvt_kd);
            d.set_target_position(static_cast<int32_t>(filtered));
            d.set_target_velocity(0);

            if (ct.logger)
            {
                LogSample sample;
                sample.t_s = static_cast<double>(impl.tick) * dt;
                sample.phase = ct.phase;
                sample.phase_t_s = ct.phase_t;
                sample.ref_raw_counts = ref_counts;
                sample.ref_filt_counts = filtered;
                sample.actual_counts = actual_counts;
                sample.motor_temp_c = motor_t;
                sample.drive_temp_c = drive_t;
                sample.error_code = err;
                ct.logger->write(sample);
            }

            ct.phase_t += dt;
            break;
        }
        }
    }

    // Publish telemetry at ~50 Hz to keep the GUI snappy without flooding.
    const int decim = std::max(1, static_cast<int>(m_profile.loop_rate_hz / 50.0));
    if (impl.tick % decim == 0)
    {
        publishTelemetry();
        // Surface live limit updates while backdrivable.
        const ControllerState s = m_state.load();
        if (s == ControllerState::Capturing || s == ControllerState::Connected)
        {
            publishJointInfo();
        }
    }
}

void ControllerWorker::recomputeState()
{
    auto &impl = *m_impl;
    if (!impl.bus_up)
    {
        return;
    }
    bool running = false, capturing = false, manual = false;
    for (const auto &ct : impl.ctl)
    {
        switch (ct.activity)
        {
        case Activity::Trajectory:
            running = true;
            break;
        case Activity::Capture:
            capturing = true;
            break;
        case Activity::Jog:
        case Activity::GoTo:
            manual = true;
            break;
        case Activity::Idle:
            break;
        }
    }
    if (m_state.load() == ControllerState::Faulted)
    {
        return;
    }
    if (running)
    {
        setState(ControllerState::Running);
    }
    else if (capturing)
    {
        setState(ControllerState::Capturing);
    }
    else if (manual)
    {
        setState(ControllerState::Jogging);
    }
    else
    {
        setState(ControllerState::Connected);
    }
}

void ControllerWorker::publishTelemetry()
{
    auto &impl = *m_impl;
    actuator_test::gui::TelemetryFrame frame;
    frame.t_s = static_cast<double>(impl.tick) * m_profile.loop_period_s();
    frame.state = m_state.load();
    frame.joints.reserve(impl.joints.size());

    const double dt = m_profile.loop_period_s();

    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        auto &jh = impl.joints[i];
        auto &ct = impl.ctl[i];
        actuator_test::gui::JointTelemetry jt;
        jt.name = jh.name;
        if (jh.driver)
        {
            auto &d = *jh.driver;
            const int bits = d.encoder_bits();
            jt.reference_deg = actuator_test::counts2deg(static_cast<int32_t>(ct.cmd_counts), bits);
            jt.actual_deg = actuator_test::counts2deg(d.actual_position(), bits);
            jt.error_deg = jt.reference_deg - jt.actual_deg;
            jt.velocity_deg_s = actuator_test::counts2deg(d.actual_velocity(), bits);

            // Acceleration: (current_velocity - previous_velocity) / dt
            if (dt > 0.0)
            {
                jt.acceleration_deg_s2 = (jt.velocity_deg_s - ct.prev_velocity_deg_s) / dt;
                ct.prev_velocity_deg_s = jt.velocity_deg_s;
            }

            jt.min_limit_deg = actuator_test::counts2deg(ct.min_counts, bits);
            jt.max_limit_deg = actuator_test::counts2deg(ct.max_counts, bits);
            const bool soft_valid = (ct.soft_max_counts > ct.soft_min_counts);
            const double soft_min_counts = soft_valid ? ct.soft_min_counts : static_cast<double>(ct.min_counts);
            const double soft_max_counts = soft_valid ? ct.soft_max_counts : static_cast<double>(ct.max_counts);
            jt.soft_min_limit_deg = actuator_test::counts2deg(static_cast<int32_t>(soft_min_counts), bits);
            jt.soft_max_limit_deg = actuator_test::counts2deg(static_cast<int32_t>(soft_max_counts), bits);

            // Limit violation detection
            jt.hard_limit_violation = jt.actual_deg < jt.min_limit_deg || jt.actual_deg > jt.max_limit_deg;
            jt.limit_violation = jt.actual_deg < jt.soft_min_limit_deg || jt.actual_deg > jt.soft_max_limit_deg;

            jt.following_error_deg = jt.error_deg;
            jt.motor_temp_c = d.has_temperature_feedback() ? d.motor_temperature() : -1;
            jt.drive_temp_c = d.has_temperature_feedback() ? d.drive_temperature() : -1;
            jt.status = d.status();
            jt.error_code = d.error_code();
            jt.fault = d.fault();
            jt.cia402_state = map_status_to_cia402_state(jt.status, jt.fault);
            jt.phase = (ct.activity == Activity::Trajectory) ? ct.phase : 0;
        }
        frame.joints.push_back(std::move(jt));
    }

    publish(std::move(frame));
}

void ControllerWorker::publishJointInfo()
{
    auto &impl = *m_impl;
    std::vector<JointInfo> infos;
    infos.reserve(impl.joints.size());
    for (std::size_t i = 0; i < impl.joints.size(); ++i)
    {
        const auto &jh = impl.joints[i];
        const auto &ct = impl.ctl[i];
        JointInfo info;
        info.name = jh.name;
        info.driver_name = jh.driver_name;
        info.model = jh.model;
        info.operation_mode_name = jh.operation_mode_name;
        info.encoder_bits = jh.driver ? jh.driver->encoder_bits() : jh.encoder_bits;
        info.selectable = jh.selectable;
        info.unavailable_reason = jh.unavailable_reason;
        info.min_limit_deg = actuator_test::counts2deg(ct.min_counts, info.encoder_bits);
        info.max_limit_deg = actuator_test::counts2deg(ct.max_counts, info.encoder_bits);
        infos.push_back(std::move(info));
    }
    std::lock_guard<std::mutex> lk(m_joints_mutex);
    m_joints = std::move(infos);
}

} // namespace actuator_test::gui
