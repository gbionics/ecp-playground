#pragma once

namespace actuator_test {

enum class TrajectoryMode {
  Sin,         ///< Fixed-frequency sinusoid about the captured mid-point.
  ChirpLinear, ///< Linear-frequency chirp (sweep) f0 -> f1 -> f0.
  ChirpLog,    ///< Logarithmic (exponential) frequency chirp.
  Triangle,    ///< Constant-velocity triangle position sweep.
  Step,        ///< Square wave (step response characterisation).
  Multisine,   ///< Sum of harmonics with Schroeder phasing (frequency ID).
  Spline,      ///< Replay a Catmull-Rom spline recorded by backdriving.
};

inline const char *mode_to_string(TrajectoryMode mode) noexcept {
  switch (mode) {
  case TrajectoryMode::Sin:
    return "sin";
  case TrajectoryMode::ChirpLinear:
    return "chirp-linear";
  case TrajectoryMode::ChirpLog:
    return "chirp-log";
  case TrajectoryMode::Triangle:
    return "triangle";
  case TrajectoryMode::Step:
    return "step";
  case TrajectoryMode::Multisine:
    return "multisine";
  case TrajectoryMode::Spline:
    return "spline";
  }
  return "?";
}

/// True when the mode is "trained" by recording a waypoint trail during the
/// idle capture phase (currently only spline).  Parametric modes only need
/// the captured min/max envelope.
inline bool mode_requires_recording(TrajectoryMode mode) noexcept {
  return mode == TrajectoryMode::Spline;
}

/// True when the trajectory is generated analytically from the captured
/// [min, max] envelope (everything except spline).
inline bool mode_is_parametric(TrajectoryMode mode) noexcept {
  return mode != TrajectoryMode::Spline;
}

} // namespace actuator_test
