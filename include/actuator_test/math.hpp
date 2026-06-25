#pragma once

#include <cmath>
#include <cstdint>

namespace actuator_test
{

inline constexpr double k_pi = 3.14159265358979323846;

inline double counts2deg(int32_t counts, int encoder_bits) noexcept
{
    const int64_t cpr = int64_t{1} << encoder_bits;
    return static_cast<double>(counts) / static_cast<double>(cpr) * 360.0;
}

inline int32_t deg2counts(double deg, int encoder_bits) noexcept
{
    const int64_t cpr = int64_t{1} << encoder_bits;
    return static_cast<int32_t>(std::llround(deg / 360.0 * static_cast<double>(cpr)));
}

class LowPass
{
public:
    LowPass(double cutoff_hz, double dt_s, double initial) noexcept
        : m_alpha(1.0 - std::exp(-2.0 * k_pi * cutoff_hz * dt_s)), m_y(initial)
    {
    }

    double step(double x) noexcept
    {
        m_y += m_alpha * (x - m_y);
        return m_y;
    }

private:
    double m_alpha;
    double m_y;
};

inline double min_jerk(double t, double T, double p0, double p1) noexcept
{
    if (T <= 0.0 || t >= T)
    {
        return p1;
    }
    if (t <= 0.0)
    {
        return p0;
    }
    const double tau = t / T;
    const double tau2 = tau * tau;
    const double tau3 = tau2 * tau;
    const double tau4 = tau3 * tau;
    const double tau5 = tau4 * tau;
    const double s = 10.0 * tau3 - 15.0 * tau4 + 6.0 * tau5;
    return p0 + (p1 - p0) * s;
}

} // namespace actuator_test
