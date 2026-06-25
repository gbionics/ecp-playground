#include "actuator_test/session.hpp"

#include "actuator_test/settings.hpp"
#include "actuator_test/terminal.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <thread>

namespace actuator_test
{

bool capture_limits_multi(std::vector<JointPlan> &plans, TrajectoryMode mode, const RuntimeProfile &profile)
{
    using clock = std::chrono::steady_clock;
    if (plans.empty())
    {
        return false;
    }

    std::printf("\n--- Capture mechanical limits for %zu joint%s (mode=%s) ---\n", plans.size(),
                plans.size() == 1 ? "" : "s", mode_to_string(mode));
    std::printf("All selected drives are IDLE; you should be able to backdrive each by hand.\n");

    if (mode == TrajectoryMode::Sin)
    {
        std::printf("Move EACH joint through its FULL range of motion (any order, any speed).\n");
        std::printf("MIN/MAX are tracked automatically per-joint. Press <Enter> when ALL joints have\n");
        std::printf("been swept end-to-end, or 'q' to abort.\n\n");
    }
    else
    {
        std::printf("Move ALL selected joints together through the trajectory you want to replay.\n");
        std::printf("Min/max are tracked AND motion is recorded at %.0f Hz for spline training.\n",
                    profile.waypoint_record_rate_hz);
        std::printf("Small dithers are filtered by decimation threshold %.2f deg.\n\n", profile.waypoint_decimation_deg);
    }

    for (auto &p : plans)
    {
        const int32_t seed = p.jh->driver->actual_position();
        p.min_counts = seed;
        p.max_counts = seed;
    }

    struct TrailFrame
    {
        std::vector<int32_t> pos;
    };

    std::vector<TrailFrame> trail;
    if (mode_requires_recording(mode))
    {
        trail.reserve(2048);
        TrailFrame f0;
        f0.pos.reserve(plans.size());
        for (auto &p : plans)
        {
            f0.pos.push_back(p.jh->driver->actual_position());
        }
        trail.push_back(std::move(f0));
    }

    RawTty tty;
    auto last_print = clock::now() - std::chrono::seconds(2);
    auto last_snap = clock::now();
    bool done = false;
    bool aborted = false;

    while (ecp::rt_app_t::instance().running() && !done && !aborted)
    {
        int c = tty.try_read();
        while (c >= 0)
        {
            if (c == 'q' || c == 'Q')
            {
                aborted = true;
                break;
            }
            if (c == '\n' || c == '\r')
            {
                done = true;
                break;
            }
            c = tty.try_read();
        }

        for (auto &p : plans)
        {
            const int32_t a = p.jh->driver->actual_position();
            p.min_counts = std::min(p.min_counts, a);
            p.max_counts = std::max(p.max_counts, a);
        }

        if (mode_requires_recording(mode))
        {
            const auto now = clock::now();
            if (std::chrono::duration<double>(now - last_snap).count() >= profile.waypoint_record_period_s())
            {
                last_snap = now;
                TrailFrame f;
                f.pos.reserve(plans.size());
                for (auto &p : plans)
                {
                    f.pos.push_back(p.jh->driver->actual_position());
                }
                trail.push_back(std::move(f));
            }
        }

        const auto now = clock::now();
        if (std::chrono::duration<double>(now - last_print).count() >= profile.status_print_interval_s)
        {
            last_print = now;
            std::printf("\n  -- live limits%s --\n", mode_requires_recording(mode) ? " + trail" : "");
            for (auto &p : plans)
            {
                const int32_t a = p.jh->driver->actual_position();
                const double range_deg = counts2deg(p.max_counts - p.min_counts, p.jh->driver->encoder_bits());
                const char *good = (range_deg >= profile.min_range_deg) ? "OK " : "...";
                std::printf("    [%s] %-16s cur=%+8.2f MIN=%+8.2f MAX=%+8.2f range=%6.2f deg\n", good,
                            p.jh->name.c_str(), counts2deg(a, p.jh->driver->encoder_bits()),
                            counts2deg(p.min_counts, p.jh->driver->encoder_bits()),
                            counts2deg(p.max_counts, p.jh->driver->encoder_bits()), range_deg);
            }
            if (mode_requires_recording(mode))
            {
                std::printf("    raw trail frames so far: %zu (~%.1f s)\n", trail.size(),
                            trail.size() * profile.waypoint_record_period_s());
            }
            std::fflush(stdout);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    std::printf("\n");
    if (aborted || !done)
    {
        return false;
    }

    std::printf("--- Captured envelopes ---\n");
    bool too_small_any = false;
    for (auto &p : plans)
    {
        const double range_deg = counts2deg(p.max_counts - p.min_counts, p.jh->driver->encoder_bits());
        std::printf("  %-16s MIN=%+8.2fdeg MAX=%+8.2fdeg range=%6.2fdeg centre=%+8.2fdeg%s\n", p.jh->name.c_str(),
                    counts2deg(p.min_counts, p.jh->driver->encoder_bits()),
                    counts2deg(p.max_counts, p.jh->driver->encoder_bits()), range_deg,
                    counts2deg((p.min_counts + p.max_counts) / 2, p.jh->driver->encoder_bits()),
                    range_deg < profile.min_range_deg ? "  <-- BELOW SAFETY FLOOR" : "");
        if (range_deg < profile.min_range_deg)
        {
            too_small_any = true;
        }
    }

    if (too_small_any)
    {
        std::fprintf(stderr, "One or more joints have range < %.2f deg. Aborting selection.\n", profile.min_range_deg);
        return false;
    }

    if (mode_requires_recording(mode))
    {
        if (trail.size() < 2)
        {
            std::fprintf(stderr, "Spline trail too short (%zu frames). Aborting selection.\n", trail.size());
            return false;
        }

        std::vector<std::size_t> kept_indices;
        kept_indices.reserve(trail.size());
        kept_indices.push_back(0);
        for (std::size_t i = 1; i + 1 < trail.size(); ++i)
        {
            bool moved = false;
            const std::size_t last = kept_indices.back();
            for (std::size_t pi = 0; pi < plans.size(); ++pi)
            {
                const double d_deg = std::fabs(counts2deg(trail[i].pos[pi] - trail[last].pos[pi],
                                                          plans[pi].jh->driver->encoder_bits()));
                if (d_deg >= profile.waypoint_decimation_deg)
                {
                    moved = true;
                    break;
                }
            }
            if (moved)
            {
                kept_indices.push_back(i);
            }
        }

        if (kept_indices.back() != trail.size() - 1)
        {
            bool moved_from_last = false;
            const std::size_t last = kept_indices.back();
            for (std::size_t pi = 0; pi < plans.size(); ++pi)
            {
                const double d_deg = std::fabs(counts2deg(trail.back().pos[pi] - trail[last].pos[pi],
                                                          plans[pi].jh->driver->encoder_bits()));
                if (d_deg >= profile.waypoint_decimation_deg)
                {
                    moved_from_last = true;
                    break;
                }
            }
            if (moved_from_last)
            {
                kept_indices.push_back(trail.size() - 1);
            }
            else
            {
                kept_indices.back() = trail.size() - 1;
            }
        }

        if (kept_indices.size() > profile.max_spline_waypoints)
        {
            std::vector<std::size_t> thinned;
            thinned.reserve(profile.max_spline_waypoints);
            const double step = static_cast<double>(kept_indices.size() - 1) /
                                static_cast<double>(profile.max_spline_waypoints - 1);
            for (std::size_t k = 0; k < profile.max_spline_waypoints; ++k)
            {
                const std::size_t idx = static_cast<std::size_t>(std::round(k * step));
                const std::size_t src = std::min(kept_indices.size() - 1, idx);
                if (thinned.empty() || thinned.back() != kept_indices[src])
                {
                    thinned.push_back(kept_indices[src]);
                }
            }
            kept_indices.swap(thinned);
        }

        if (kept_indices.size() < profile.min_spline_waypoints)
        {
            std::fprintf(stderr,
                         "Spline has only %zu waypoint(s) after decimation (need >= %zu). "
                         "Move the joints more, or use sin mode. Aborting selection.\n",
                         kept_indices.size(), profile.min_spline_waypoints);
            return false;
        }

        std::printf("--- Recorded spline (%zu waypoints from %zu trail frames) ---\n", kept_indices.size(), trail.size());
        for (std::size_t pi = 0; pi < plans.size(); ++pi)
        {
            plans[pi].waypoint_counts.clear();
            plans[pi].waypoint_counts.reserve(kept_indices.size());
            for (std::size_t k : kept_indices)
            {
                plans[pi].waypoint_counts.push_back(trail[k].pos[pi]);
            }
        }
    }

    return true;
}

void run_trajectory_multi(std::vector<JointPlan> &plans, TrajectoryMode mode, const RuntimeProfile &profile)
{
    using clock = std::chrono::steady_clock;
    if (plans.empty())
    {
        return;
    }

    const std::string log_dir = make_run_log_dir(profile);

    if (mode_requires_recording(mode))
    {
        const std::size_t n = plans.front().waypoint_counts.size();
        for (const auto &p : plans)
        {
            if (p.waypoint_counts.size() != n || n < profile.min_spline_waypoints)
            {
                std::fprintf(stderr,
                             "internal error: spline waypoint mismatch (joint '%s' has %zu, expected %zu, min %zu).\n",
                             p.jh->name.c_str(), p.waypoint_counts.size(), n, profile.min_spline_waypoints);
                return;
            }
        }

        std::vector<double> knot_times(n, 0.0);
        for (std::size_t i = 1; i < n; ++i)
        {
            double seg = 0.0;
            for (const auto &p : plans)
            {
                const double d_deg = std::fabs(
                    counts2deg(p.waypoint_counts[i] - p.waypoint_counts[i - 1], p.jh->driver->encoder_bits()));
                seg = std::max(seg, d_deg / profile.max_approach_speed_deg_s);
            }
            seg = std::max(seg, profile.min_segment_seconds);
            knot_times[i] = knot_times[i - 1] + seg;
        }

        for (auto &p : plans)
        {
            p.traj = std::make_unique<SplineTrajectory>(knot_times, p.waypoint_counts, p.jh->driver->encoder_bits());
        }
    }
    else
    {
        for (auto &p : plans)
        {
            const double centre = static_cast<double>((p.min_counts + p.max_counts) / 2);
            const int32_t half_range = (p.max_counts - p.min_counts) / 2;
            const double amp = profile.traj_safety_factor * static_cast<double>(half_range);
            p.traj = make_parametric_trajectory(mode, centre, amp, profile, p.jh->driver->encoder_bits());
        }
    }

    std::printf("\n--- Trajectory plan (%zu joint%s, mode=%s, %.0fHz loop / %.1fHz LPF) ---\n", plans.size(),
                plans.size() == 1 ? "" : "s", mode_to_string(mode), profile.loop_rate_hz, profile.lpf_cutoff_hz);
    for (auto &p : plans)
    {
        const double target_deg = counts2deg(static_cast<int32_t>(p.traj->approach_target()), p.jh->driver->encoder_bits());
        const int32_t preview_counts = p.jh->driver->actual_position();
        const double preview_deg = counts2deg(preview_counts, p.jh->driver->encoder_bits());
        const double preview_dist = std::fabs(target_deg - preview_deg);
        const double preview_t = std::max(profile.approach_seconds, preview_dist / profile.max_approach_speed_deg_s);

        if (mode == TrajectoryMode::Sin)
        {
            const double centre_deg = counts2deg((p.min_counts + p.max_counts) / 2, p.jh->driver->encoder_bits());
            const double amp_deg = counts2deg(
                static_cast<int32_t>(profile.traj_safety_factor * ((p.max_counts - p.min_counts) / 2)),
                p.jh->driver->encoder_bits());
            std::printf("  %-16s drv=%-10s mode=%-6s current=%+7.2fdeg centre=%+7.2fdeg amp=%.2fdeg @ %.2fHz approach~%.2fs KP=%d KD=%d\n",
                        p.jh->name.c_str(), p.jh->driver_name.c_str(), p.jh->operation_mode_name.c_str(), preview_deg,
                    centre_deg, amp_deg, profile.traj_freq_hz, preview_t, p.jh->pvt_kp, p.jh->pvt_kd);
        }
        else
        {
            auto *sp = static_cast<SplineTrajectory *>(p.traj.get());
            std::printf("  %-16s drv=%-10s mode=%-6s current=%+7.2fdeg w0=%+7.2fdeg waypoints=%zu one-way=%.2fs approach~%.2fs KP=%d KD=%d\n",
                        p.jh->name.c_str(), p.jh->driver_name.c_str(), p.jh->operation_mode_name.c_str(), preview_deg,
                        target_deg, sp->waypoint_count(), sp->total_one_way_seconds(), preview_t, p.jh->pvt_kp,
                        p.jh->pvt_kd);
        }
    }

    std::printf("  loop rate = %.0f Hz, LPF cutoff = %.1f Hz, pre-hold = %.2fs, max approach speed = %.1f deg/s\n",
                profile.loop_rate_hz, profile.lpf_cutoff_hz, profile.pre_ramp_hold_seconds,
                profile.max_approach_speed_deg_s);
    std::printf("\nPress <Enter> to engage all drives and start, or 'q'+<Enter> to abort: ");
    std::fflush(stdout);

    {
        std::string line;
        if (!std::getline(std::cin, line))
        {
            return;
        }
        if (line == "q" || line == "Q")
        {
            return;
        }
    }

    for (auto &p : plans)
    {
        auto &d = *p.jh->driver;
        d.set_target_velocity(0);
        d.set_target_position(d.actual_position());
        d.apply_runtime_gains(p.jh->pvt_kp, p.jh->pvt_kd);
        d.update_operation_mode(p.jh->operation_mode_code);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    for (auto &p : plans)
    {
        p.start_counts = p.jh->driver->actual_position();
        const double start_deg = counts2deg(p.start_counts, p.jh->driver->encoder_bits());
        const double target_deg = counts2deg(static_cast<int32_t>(p.traj->approach_target()), p.jh->driver->encoder_bits());
        const double dist_deg = std::fabs(target_deg - start_deg);
        p.approach_T = std::max(profile.approach_seconds, dist_deg / profile.max_approach_speed_deg_s);
        p.phase = ControlPhase::Hold;
        p.phase_t = 0.0;
        p.lpf = std::make_unique<LowPass>(profile.lpf_cutoff_hz, profile.loop_period_s(),
                                          static_cast<double>(p.start_counts));

        std::printf("  engaged '%s': start=%+.2fdeg, ramping to traj start=%+.2fdeg over %.2fs\n", p.jh->name.c_str(),
                    start_deg, target_deg, p.approach_T);

        if (!log_dir.empty())
        {
            const std::string path = log_dir + "/" + p.jh->name + ".csv";
            p.csv_logger = std::make_unique<JointCsvLogger>();
            if (p.csv_logger->open(path, profile, *p.jh, p, *p.traj))
            {
                std::printf("    logging '%s' to %s\n", p.jh->name.c_str(), path.c_str());
            }
            else
            {
                p.csv_logger.reset();
                std::fprintf(stderr, "warning: cannot open log file '%s' -- skipping log for %s\n", path.c_str(),
                             p.jh->name.c_str());
            }
        }
    }

    RawTty tty;
    const auto t0 = clock::now();
    int print_div = 0;
    bool aborted = false;
    std::string abort_reason;
    std::string abort_joint;

    std::printf("\nRunning at %.0f Hz. Press 'q' to stop, '<Enter>' to print status.\n", profile.loop_rate_hz);

    int64_t tick = 0;
    while (ecp::rt_app_t::instance().running())
    {
        int c = tty.try_read();
        while (c >= 0)
        {
            if (c == 'q' || c == 'Q')
            {
                std::printf("\nStop requested.\n");
                aborted = true;
                abort_reason = "user quit";
            }
            else if (c == '\n' || c == '\r')
            {
                print_div = 0;
            }
            c = tty.try_read();
        }
        if (aborted)
        {
            break;
        }

        for (auto &p : plans)
        {
            std::string reason;
            if (safety_violated(*p.jh->driver, profile, p.jh->name, p.safety, reason))
            {
                ECERR("Safety abort on '%s': %s\n", p.jh->name.c_str(), reason.c_str());
                aborted = true;
                abort_reason = reason;
                abort_joint = p.jh->name;
                break;
            }
        }
        if (aborted)
        {
            break;
        }

        for (auto &p : plans)
        {
            double ref_counts = 0.0;
            if (p.phase == ControlPhase::Hold)
            {
                ref_counts = static_cast<double>(p.start_counts);
                if (p.phase_t >= profile.pre_ramp_hold_seconds)
                {
                    p.phase = ControlPhase::Approach;
                    p.phase_t = 0.0;
                    std::printf("\n[%s] hold complete -- starting min-jerk approach.\n", p.jh->name.c_str());
                }
            }
            else if (p.phase == ControlPhase::Approach)
            {
                ref_counts = min_jerk(p.phase_t, p.approach_T, static_cast<double>(p.start_counts), p.traj->approach_target());
                if (p.phase_t >= p.approach_T)
                {
                    p.phase = ControlPhase::Trajectory;
                    p.phase_t = 0.0;
                    std::printf("\n[%s] approach complete -- starting trajectory.\n", p.jh->name.c_str());
                }
            }
            else
            {
                ref_counts = p.traj->sample(p.phase_t);
            }

            double filtered = p.lpf->step(ref_counts);
            filtered = std::clamp(filtered, static_cast<double>(p.min_counts), static_cast<double>(p.max_counts));

            const int32_t target = static_cast<int32_t>(filtered);
            auto &d = *p.jh->driver;
            const int32_t actual_counts = d.actual_position();
            const int16_t motor_t = d.has_temperature_feedback() ? d.motor_temperature() : -1;
            const int16_t drive_t = d.has_temperature_feedback() ? d.drive_temperature() : -1;
            const uint16_t err = d.error_code();

            d.apply_runtime_gains(p.jh->pvt_kp, p.jh->pvt_kd);
            d.set_target_position(target);
            d.set_target_velocity(0);

            LogSample sample;
            sample.t_s = static_cast<double>(tick) * profile.loop_period_s();
            sample.phase = static_cast<int>(p.phase);
            sample.phase_t_s = p.phase_t;
            sample.ref_raw_counts = ref_counts;
            sample.ref_filt_counts = filtered;
            sample.actual_counts = actual_counts;
            sample.motor_temp_c = motor_t;
            sample.drive_temp_c = drive_t;
            sample.error_code = err;
            if (p.csv_logger)
            {
                p.csv_logger->write(sample);
            }

            p.phase_t += profile.loop_period_s();
        }

        if (++print_div >= static_cast<int>(0.5 * profile.loop_rate_hz))
        {
            print_div = 0;
            std::printf("[%6.2fs]", std::chrono::duration<double>(clock::now() - t0).count());
            for (auto &p : plans)
            {
                const double actual_deg = counts2deg(p.jh->driver->actual_position(), p.jh->driver->encoder_bits());
                if (p.jh->driver->has_temperature_feedback())
                {
                    std::printf("  %s:ph%d act=%+6.2f motor=%dC", p.jh->name.c_str(), static_cast<int>(p.phase), actual_deg,
                                static_cast<int>(p.jh->driver->motor_temperature()));
                }
                else
                {
                    std::printf("  %s:ph%d act=%+6.2f", p.jh->name.c_str(), static_cast<int>(p.phase), actual_deg);
                }
            }
            std::printf("\n");
            std::fflush(stdout);
        }

        ++tick;
        std::this_thread::sleep_until(
            t0 + std::chrono::microseconds(static_cast<int64_t>(tick * profile.loop_period_s() * 1e6)));
    }

    for (auto &p : plans)
    {
        auto &d = *p.jh->driver;
        d.set_target_position(d.actual_position());
        d.set_target_velocity(0);
        d.idle();

        if (p.csv_logger && p.csv_logger->is_open())
        {
            const std::string log_path = p.csv_logger->path();
            p.csv_logger->close();
            std::printf("  wrote log: %s\n", log_path.c_str());
        }
        p.csv_logger.reset();
    }

    if (aborted)
    {
        std::printf("Trajectory aborted (%s%s%s).\n", abort_joint.empty() ? "" : abort_joint.c_str(),
                    abort_joint.empty() ? "" : ": ", abort_reason.c_str());
    }
    else
    {
        std::printf("Trajectory loop exited (rt_app stop).\n");
    }
}

} // namespace actuator_test
