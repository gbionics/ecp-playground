#include "actuator_test/trajectory.hpp"

#include <algorithm>
#include <cmath>

namespace actuator_test {

SinTrajectory::SinTrajectory(double centre_counts, double amp_counts,
                             double freq_hz, double safety_factor,
                             int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts), m_freq(freq_hz),
      m_safety_factor(safety_factor), m_enc_bits(encoder_bits) {}

double SinTrajectory::approach_target() const noexcept { return m_centre; }

double SinTrajectory::sample(double t_s) const noexcept {
  return m_centre + m_amp * std::sin(2.0 * k_pi * m_freq * t_s);
}

void SinTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, sin\n";
  os << "# sin_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# sin_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# sin_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# sin_amp_deg, " << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits)
     << "\n";
  os << "# sin_freq_hz, " << m_freq << "\n";
  os << "# traj_safety_factor, " << m_safety_factor << "\n";
}

// ---------------------------------------------------------------------------
//  ChirpTrajectory
// ---------------------------------------------------------------------------

ChirpTrajectory::ChirpTrajectory(double centre_counts, double amp_counts,
                                 double f0_hz, double f1_hz,
                                 double sweep_seconds, bool logarithmic,
                                 int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts), m_f0(std::max(1e-6, f0_hz)),
      m_f1(std::max(1e-6, f1_hz)), m_sweep(std::max(1e-3, sweep_seconds)),
      m_log(logarithmic), m_enc_bits(encoder_bits) {}

double ChirpTrajectory::approach_target() const noexcept {
  // sin(phase(0)) = sin(0) = 0, so the chirp starts at the centre.
  return m_centre;
}

double ChirpTrajectory::sweep_integral(double x) const noexcept {
  // F(x) = integral over [0, x] of the up-sweep instantaneous frequency.
  const double T = m_sweep;
  if (!m_log) {
    return m_f0 * x + (m_f1 - m_f0) * x * x / (2.0 * T);
  }

  const double r = m_f1 / m_f0;
  if (std::fabs(r - 1.0) < 1e-9) {
    return m_f0 * x;
  }
  return m_f0 * T / std::log(r) * (std::pow(r, x / T) - 1.0);
}

double ChirpTrajectory::phase_at(double t_s) const noexcept {
  const double T = m_sweep;
  const double period = 2.0 * T;
  double t = t_s;
  if (t < 0.0) {
    t = 0.0;
  }

  const double n = std::floor(t / period);
  const double u = t - n * period;

  const double F_T = sweep_integral(T);
  const double phase_period = 2.0 * k_pi * 2.0 * F_T; // up + mirrored down.

  double partial = 0.0;
  if (u <= T) {
    partial = 2.0 * k_pi * sweep_integral(u);
  } else {
    const double y = u - T; // [0, T] into the mirrored down-sweep.
    partial = 2.0 * k_pi * (2.0 * F_T - sweep_integral(T - y));
  }

  return n * phase_period + partial;
}

double ChirpTrajectory::sample(double t_s) const noexcept {
  return m_centre + m_amp * std::sin(phase_at(t_s));
}

void ChirpTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, " << (m_log ? "chirp_log" : "chirp_linear") << "\n";
  os << "# chirp_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# chirp_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# chirp_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# chirp_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# chirp_f0_hz, " << m_f0 << "\n";
  os << "# chirp_f1_hz, " << m_f1 << "\n";
  os << "# chirp_sweep_seconds, " << m_sweep << "\n";
  os << "# chirp_full_period_s, " << (2.0 * m_sweep) << "\n";
}

// ---------------------------------------------------------------------------
//  TriangleTrajectory
// ---------------------------------------------------------------------------

TriangleTrajectory::TriangleTrajectory(double centre_counts, double amp_counts,
                                       double cycle_seconds,
                                       int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts),
      m_cycle(std::max(1e-3, cycle_seconds)), m_enc_bits(encoder_bits) {}

double TriangleTrajectory::approach_target() const noexcept { return m_centre; }

double TriangleTrajectory::sample(double t_s) const noexcept {
  // (2/pi) * asin(sin(theta)) is a unit triangle starting at 0 and rising.
  const double theta = 2.0 * k_pi * t_s / m_cycle;
  const double tri = (2.0 / k_pi) * std::asin(std::sin(theta));
  return m_centre + m_amp * tri;
}

void TriangleTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, triangle\n";
  os << "# triangle_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# triangle_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# triangle_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# triangle_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# triangle_cycle_seconds, " << m_cycle << "\n";
}

// ---------------------------------------------------------------------------
//  StepTrajectory
// ---------------------------------------------------------------------------

StepTrajectory::StepTrajectory(double centre_counts, double amp_counts,
                               double cycle_seconds, int encoder_bits) noexcept
    : m_centre(centre_counts), m_amp(amp_counts),
      m_cycle(std::max(1e-3, cycle_seconds)), m_enc_bits(encoder_bits) {}

double StepTrajectory::approach_target() const noexcept {
  // First half-cycle commands +amp; ramp the approach there to avoid a step
  // at engage (the LPF still shapes subsequent edges).
  return m_centre + m_amp;
}

double StepTrajectory::sample(double t_s) const noexcept {
  const double phase = std::fmod(t_s, m_cycle) / m_cycle; // [0, 1)
  const double sign = (phase < 0.5) ? 1.0 : -1.0;
  return m_centre + m_amp * sign;
}

void StepTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, step\n";
  os << "# step_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# step_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# step_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# step_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# step_cycle_seconds, " << m_cycle << "\n";
}

// ---------------------------------------------------------------------------
//  MultisineTrajectory
// ---------------------------------------------------------------------------

MultisineTrajectory::MultisineTrajectory(double centre_counts,
                                         double amp_counts, double base_freq_hz,
                                         int harmonics, int encoder_bits)
    : m_centre(centre_counts), m_amp(amp_counts),
      m_base_freq(std::max(1e-6, base_freq_hz)),
      m_harmonics(std::max(1, harmonics)), m_enc_bits(encoder_bits) {
  // Schroeder phases keep the crest factor low so the summed amplitude stays
  // within the captured envelope.
  m_phases.resize(static_cast<std::size_t>(m_harmonics));
  for (int k = 1; k <= m_harmonics; ++k) {
    m_phases[static_cast<std::size_t>(k - 1)] =
        -k_pi * static_cast<double>(k) * static_cast<double>(k - 1) /
        static_cast<double>(m_harmonics);
  }
}

double MultisineTrajectory::approach_target() const noexcept {
  return sample(0.0);
}

double MultisineTrajectory::sample(double t_s) const noexcept {
  double acc = 0.0;
  for (int k = 1; k <= m_harmonics; ++k) {
    const double f = m_base_freq * static_cast<double>(k);
    acc += std::sin(2.0 * k_pi * f * t_s +
                    m_phases[static_cast<std::size_t>(k - 1)]);
  }
  return m_centre + (m_amp / static_cast<double>(m_harmonics)) * acc;
}

void MultisineTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, multisine\n";
  os << "# multisine_centre_counts, " << static_cast<int64_t>(m_centre) << "\n";
  os << "# multisine_amp_counts, " << static_cast<int64_t>(m_amp) << "\n";
  os << "# multisine_centre_deg, "
     << counts2deg(static_cast<int32_t>(m_centre), m_enc_bits) << "\n";
  os << "# multisine_amp_deg, "
     << counts2deg(static_cast<int32_t>(m_amp), m_enc_bits) << "\n";
  os << "# multisine_base_freq_hz, " << m_base_freq << "\n";
  os << "# multisine_harmonics, " << m_harmonics << "\n";
}

std::unique_ptr<Trajectory>
make_parametric_trajectory(TrajectoryMode mode, double centre, double amp,
                           const RuntimeProfile &profile, int encoder_bits) {
  switch (mode) {
  case TrajectoryMode::Sin:
    return std::make_unique<SinTrajectory>(centre, amp, profile.traj_freq_hz,
                                           profile.traj_safety_factor,
                                           encoder_bits);
  case TrajectoryMode::ChirpLinear:
    return std::make_unique<ChirpTrajectory>(
        centre, amp, profile.chirp_f0_hz, profile.chirp_f1_hz,
        profile.chirp_sweep_seconds, false, encoder_bits);
  case TrajectoryMode::ChirpLog:
    return std::make_unique<ChirpTrajectory>(
        centre, amp, profile.chirp_f0_hz, profile.chirp_f1_hz,
        profile.chirp_sweep_seconds, true, encoder_bits);
  case TrajectoryMode::Triangle:
    return std::make_unique<TriangleTrajectory>(
        centre, amp, profile.triangle_cycle_seconds, encoder_bits);
  case TrajectoryMode::Step:
    return std::make_unique<StepTrajectory>(
        centre, amp, profile.step_cycle_seconds, encoder_bits);
  case TrajectoryMode::Multisine:
    return std::make_unique<MultisineTrajectory>(
        centre, amp, profile.multisine_base_hz, profile.multisine_harmonics,
        encoder_bits);
  case TrajectoryMode::Spline:
    break; // Spline is built from recorded waypoints, not here.
  }
  return std::make_unique<SinTrajectory>(centre, amp, profile.traj_freq_hz,
                                         profile.traj_safety_factor,
                                         encoder_bits);
}

SplineTrajectory::SplineTrajectory(std::vector<double> knot_times,
                                   std::vector<int32_t> waypoint_counts,
                                   int encoder_bits)
    : m_t(std::move(knot_times)),
      m_w(waypoint_counts.begin(), waypoint_counts.end()),
      m_enc_bits(encoder_bits) {
  const std::size_t n = m_w.size();
  m_T = (n > 0) ? m_t.back() : 0.0;
  m_m.assign(n, 0.0);
  if (n >= 2) {
    m_m.front() = (m_w[1] - m_w[0]) / (m_t[1] - m_t[0]);
    m_m.back() = (m_w[n - 1] - m_w[n - 2]) / (m_t[n - 1] - m_t[n - 2]);
    for (std::size_t i = 1; i + 1 < n; ++i) {
      m_m[i] = (m_w[i + 1] - m_w[i - 1]) / (m_t[i + 1] - m_t[i - 1]);
    }
  }
}

double SplineTrajectory::approach_target() const noexcept {
  return m_w.empty() ? 0.0 : m_w.front();
}

double SplineTrajectory::sample(double t_s) const noexcept {
  if (m_w.empty()) {
    return 0.0;
  }
  if (m_w.size() == 1 || m_T <= 0.0) {
    return m_w.front();
  }

  const double period = 2.0 * m_T;
  double u = std::fmod(t_s, period);
  if (u < 0.0) {
    u += period;
  }
  const double s = (u <= m_T) ? u : (period - u);

  std::size_t i = 0;
  if (s >= m_t.back()) {
    i = m_w.size() - 2;
  } else {
    while (i + 1 < m_t.size() && s >= m_t[i + 1]) {
      ++i;
    }
  }

  const double dt = m_t[i + 1] - m_t[i];
  const double tau = (dt > 0.0) ? (s - m_t[i]) / dt : 0.0;
  const double tau2 = tau * tau;
  const double tau3 = tau2 * tau;
  const double h00 = 2.0 * tau3 - 3.0 * tau2 + 1.0;
  const double h10 = tau3 - 2.0 * tau2 + tau;
  const double h01 = -2.0 * tau3 + 3.0 * tau2;
  const double h11 = tau3 - tau2;

  return h00 * m_w[i] + h10 * dt * m_m[i] + h01 * m_w[i + 1] +
         h11 * dt * m_m[i + 1];
}

void SplineTrajectory::describe_csv(std::ostream &os) const {
  os << "# trajectory_mode, spline\n";
  os << "# spline_kind, cardinal_catmull_rom\n";
  os << "# spline_waypoint_count, " << m_w.size() << "\n";
  os << "# spline_total_period_s, " << (2.0 * m_T) << "\n";
  os << "# spline_one_way_seconds, " << m_T << "\n";
  os << "# spline_knot_times_s,";
  for (double t : m_t) {
    os << ' ' << t;
  }
  os << "\n# spline_waypoint_counts,";
  for (double w : m_w) {
    os << ' ' << static_cast<int64_t>(w);
  }
  os << "\n# spline_waypoint_deg,";
  for (double w : m_w) {
    os << ' ' << counts2deg(static_cast<int32_t>(w), m_enc_bits);
  }
  os << '\n';
}

double SplineTrajectory::total_one_way_seconds() const noexcept { return m_T; }

std::size_t SplineTrajectory::waypoint_count() const noexcept {
  return m_w.size();
}

} // namespace actuator_test
