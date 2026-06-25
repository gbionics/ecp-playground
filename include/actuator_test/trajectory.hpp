#pragma once

#include "actuator_test/math.hpp"
#include "actuator_test/settings.hpp"
#include "actuator_test/types.hpp"

#include <cstdint>
#include <memory>
#include <ostream>
#include <vector>

namespace actuator_test
{

class Trajectory
{
public:
    virtual ~Trajectory() = default;
    virtual double approach_target() const noexcept = 0;
    virtual double sample(double t_s) const noexcept = 0;
    virtual void describe_csv(std::ostream &os) const = 0;
};

class SinTrajectory final : public Trajectory
{
public:
    SinTrajectory(double centre_counts, double amp_counts, double freq_hz, double safety_factor,
                  int encoder_bits) noexcept;

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

private:
    double m_centre;
    double m_amp;
    double m_freq;
    double m_safety_factor;
    int m_enc_bits;
};

/// Frequency-sweep (chirp) about a centre with mirrored-time playback so the
/// frequency ramps f0 -> f1 -> f0 continuously and indefinitely.  Supports
/// both linear and logarithmic (exponential) frequency progression.  The
/// instantaneous frequency profile is a continuous triangle wave, so the
/// generated position is C1-continuous (no velocity step at turnarounds).
class ChirpTrajectory final : public Trajectory
{
public:
    ChirpTrajectory(double centre_counts, double amp_counts, double f0_hz, double f1_hz, double sweep_seconds,
                    bool logarithmic, int encoder_bits) noexcept;

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

private:
    double phase_at(double t_s) const noexcept;
    double sweep_integral(double x) const noexcept; ///< F(x) = integral of f over [0, x] of the up-sweep.

    double m_centre;
    double m_amp;
    double m_f0;
    double m_f1;
    double m_sweep;
    bool m_log;
    int m_enc_bits;
};

/// Constant-velocity triangle sweep between (centre - amp) and (centre + amp).
/// Starts at the centre rising, useful for slow full-range mechanical sweeps.
class TriangleTrajectory final : public Trajectory
{
public:
    TriangleTrajectory(double centre_counts, double amp_counts, double cycle_seconds, int encoder_bits) noexcept;

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

private:
    double m_centre;
    double m_amp;
    double m_cycle;
    int m_enc_bits;
};

/// Square-wave step trajectory alternating between centre +/- amp every half
/// cycle.  The reference low-pass filter shapes the commanded edge, so this
/// yields a repeatable step-response characterisation input.
class StepTrajectory final : public Trajectory
{
public:
    StepTrajectory(double centre_counts, double amp_counts, double cycle_seconds, int encoder_bits) noexcept;

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

private:
    double m_centre;
    double m_amp;
    double m_cycle;
    int m_enc_bits;
};

/// Sum-of-sines (multisine) trajectory with Schroeder phasing to minimise the
/// crest factor.  Excites several frequencies simultaneously, which is ideal
/// for one-shot frequency-response identification.
class MultisineTrajectory final : public Trajectory
{
public:
    MultisineTrajectory(double centre_counts, double amp_counts, double base_freq_hz, int harmonics,
                        int encoder_bits);

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

private:
    double m_centre;
    double m_amp;
    double m_base_freq;
    int m_harmonics;
    int m_enc_bits;
    std::vector<double> m_phases;
};

/// Build an analytic (non-recorded) trajectory for one joint from its captured
/// [centre, amplitude] envelope.  Shared by the console session builder and the
/// GUI controller so both map each mode to a generator identically.  Spline is
/// recorded, not parametric, so passing TrajectoryMode::Spline falls back to a
/// sinusoid (callers build splines explicitly from recorded waypoints).
std::unique_ptr<Trajectory> make_parametric_trajectory(TrajectoryMode mode, double centre, double amp,
                                                       const RuntimeProfile &profile, int encoder_bits);

class SplineTrajectory final : public Trajectory
{
public:
    SplineTrajectory(std::vector<double> knot_times, std::vector<int32_t> waypoint_counts, int encoder_bits);

    double approach_target() const noexcept override;
    double sample(double t_s) const noexcept override;
    void describe_csv(std::ostream &os) const override;

    double total_one_way_seconds() const noexcept;
    std::size_t waypoint_count() const noexcept;

private:
    std::vector<double> m_t;
    std::vector<double> m_w;
    std::vector<double> m_m;
    double m_T = 0.0;
    int m_enc_bits;
};

} // namespace actuator_test
