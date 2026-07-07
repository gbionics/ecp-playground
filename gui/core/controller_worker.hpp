// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// ControllerWorker owns every EtherCAT interaction on a dedicated real-time
// thread.  The GUI never touches the bus: it posts Commands and polls
// snapshots/events.  This keeps the 1 kHz control loop free of Qt and free of
// GUI-induced jitter.

#pragma once

#include "core/commands.hpp"
#include "core/telemetry.hpp"

#include "actuator_test/settings.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace actuator_test::gui {

/// Static description of one enumerated joint, surfaced to the GUI after a
/// successful connect so panels can populate themselves.
struct JointInfo {
  std::string name;
  std::string driver_name;
  std::string model;
  std::string operation_mode_name;
  int encoder_bits = 17;
  bool selectable = false;
  std::string unavailable_reason;
  double min_limit_deg = 0.0;
  double max_limit_deg = 0.0;
};

/// Low-frequency notification from the worker to the GUI (logs, state changes,
/// errors).  Drained by the GUI poll timer.
struct WorkerEvent {
  enum class Kind {
    Log,
    StateChanged,
    Error,
  };

  Kind kind = Kind::Log;
  std::string message;
  ControllerState state = ControllerState::Disconnected;
};

/// The threading bridge between the GUI and the control loop.  Construct it,
/// `start()` the thread, then drive it entirely through `post()` / `snapshot()`
/// / `drainEvents()` / `joints()`.
class ControllerWorker {
public:
  explicit ControllerWorker(RuntimeProfile profile);
  ~ControllerWorker();

  ControllerWorker(const ControllerWorker &) = delete;
  ControllerWorker &operator=(const ControllerWorker &) = delete;

  /// Spawn the real-time thread.  Must be called once before posting.
  void start();

  /// Thread-safe: enqueue a command for the control thread.
  void post(Command cmd);

  /// Thread-safe: copy the most recent telemetry frame.
  TelemetryFrame snapshot() const;

  /// Thread-safe: take and clear the pending events.
  std::vector<WorkerEvent> drainEvents();

  /// Thread-safe: copy the currently enumerated joints (empty until connected).
  std::vector<JointInfo> joints() const;

  /// Thread-safe: current high-level state.
  ControllerState state() const noexcept;

  // Implementation detail; public only so the control thread's helpers can
  // see the EtherCAT-owning state.  Never touched by the GUI thread.
  struct Impl;

private:
  // --- thread plumbing ---------------------------------------------------
  void run();                    ///< Worker thread entry point.
  bool popCommand(Command &out); ///< Non-blocking dequeue (control loop).
  void waitForCommand();         ///< Block until a command arrives (idle).
  void pushEvent(WorkerEvent ev);
  void log(std::string msg);
  void error(std::string msg);
  void setState(ControllerState s, std::string reason = {});
  void publish(TelemetryFrame frame);

  // --- control-thread routines (only run on the worker thread) -----------
  void controlTick();
  void engageJoint(std::size_t i);
  void idleJoint(std::size_t i);
  void recomputeState();
  void publishTelemetry();
  void publishJointInfo();
  void stopRecording();

  void handleCommand(const ConnectCommand &c);
  void handleCommand(const DisconnectCommand &c);
  void handleCommand(const JogCommand &c);
  void handleCommand(const GoToCommand &c);
  void handleCommand(const CaptureLimitsCommand &c);
  void handleCommand(const SetLimitsCommand &c);
  void handleCommand(const ResetLimitsCommand &c);
  void handleCommand(const StartTrajectoryCommand &c);
  void handleCommand(const StopCommand &c);
  void handleCommand(const PauseCommand &c);
  void handleCommand(const RecordCommand &c);
  void handleCommand(const ShutdownCommand &c);

  std::unique_ptr<Impl> m_impl;

  RuntimeProfile m_profile;

  // --- cross-thread channels --------------------------------------------
  mutable std::mutex m_cmd_mutex;
  std::condition_variable m_cmd_cv;
  std::deque<Command> m_commands;

  mutable std::mutex m_snap_mutex;
  TelemetryFrame m_snapshot;

  mutable std::mutex m_event_mutex;
  std::vector<WorkerEvent> m_events;

  mutable std::mutex m_joints_mutex;
  std::vector<JointInfo> m_joints;

  std::atomic<ControllerState> m_state{ControllerState::Disconnected};
  std::atomic<bool> m_shutdown{false};
  std::thread m_thread;
};

} // namespace actuator_test::gui
