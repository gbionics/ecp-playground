// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Locked-rotor characterisation test: with the shaft mechanically blocked,
// drives a single joint through a programmable current sweep (bypassing
// position/velocity feedback, same as the manual "current test" in JogPanel)
// and records commanded/actual current and output torque at each step. This
// is the standard way to verify a motor's torque constant (Kt) and check for
// current/torque non-linearities without any actual motion.

#pragma once

#include "actuator_test/external_daq.hpp"
#include "core/controller_worker.hpp"
#include "core/telemetry.hpp"

#include <QDialog>
#include <QString>
#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
class QTableWidget;
class QCheckBox;
class QLineEdit;
class QScrollBar;
QT_END_NAMESPACE

namespace actuator_test::gui {

class StripChart;

class LockedRotorTestDialog : public QDialog {
  Q_OBJECT

public:
  explicit LockedRotorTestDialog(QWidget *parent = nullptr);

  /// Refresh the joint selector (call whenever the bus (re)enumerates).
  void setJoints(const std::vector<JointInfo> &joints);

  /// Feed one telemetry frame; while a sweep is running this drives the step
  /// timing, live charts and per-step averaging. Safe to call at all times.
  void appendTelemetry(const TelemetryFrame &frame);

  /// True while a sweep is actively stepping through current levels.
  bool isRunning() const { return m_running; }

signals:
  /// Commands a constant current setpoint (A) on `joint`. When `release` is
  /// true the joint's current-control test is released back to a held
  /// position and `target_current_a` is ignored; otherwise the value is
  /// commanded verbatim, including a genuine 0 A step.
  void currentSetpointRequested(std::size_t joint, double target_current_a,
                                bool release);

private:
  struct StepResult {
    double commanded_a = 0.0;
    double avg_current_a = 0.0;
    double avg_torque_nm = 0.0;
    double peak_current_a = 0.0;
    double peak_torque_nm = 0.0;
    double ext_avg_torque_nm = 0.0; ///< External DAQ verification (if enabled).
    double ext_avg_speed_rpm = 0.0;
  };

  void onStartClicked();
  void onStopClicked();
  void buildSteps();
  void finishStep();
  void beginStep(std::size_t index);
  void stopTest(const QString &reason);
  void appendResultRow(int step_number, const StepResult &r);
  void exportResultsCsv();
  std::size_t selectedJoint() const;
  void updateStartEnabled();
  void updateExternalDaqAvailability();

  /// Applies the polarity inversion (if the checkbox is checked) so the
  /// actually-commanded value matches what's shown in the chart/table.
  double appliedCurrentA(double requested_a) const;

  QComboBox *m_joint_combo = nullptr;
  QDoubleSpinBox *m_start_spin = nullptr;
  QDoubleSpinBox *m_end_spin = nullptr;
  QDoubleSpinBox *m_step_spin = nullptr;
  QDoubleSpinBox *m_dwell_spin = nullptr;
  QCheckBox *m_return_sweep_check = nullptr;
  QCheckBox *m_invert_current_check = nullptr;
  QCheckBox *m_confirm_check = nullptr;
  QPushButton *m_start_btn = nullptr;
  QPushButton *m_stop_btn = nullptr;
  QPushButton *m_export_btn = nullptr;
  QLabel *m_status_label = nullptr;
  QTableWidget *m_results_table = nullptr;
  StripChart *m_current_chart = nullptr;
  StripChart *m_torque_chart = nullptr;
  QLabel *m_current_value_label = nullptr;
  QLabel *m_torque_value_label = nullptr;
  StripChart *m_iv_chart = nullptr;
  QScrollBar *m_time_scrollbar = nullptr;
  QPushButton *m_live_btn = nullptr;
  QDoubleSpinBox *m_window_spin = nullptr;
  bool m_scrub_updating = false;

  QCheckBox *m_ext_daq_check = nullptr;
  QCheckBox *m_ext_invert_sign_check = nullptr;
  QLineEdit *m_ext_analog_edit = nullptr;
  QLineEdit *m_ext_digital_a_edit = nullptr;
  QLineEdit *m_ext_digital_b_edit = nullptr;
  QLabel *m_ext_daq_status_label = nullptr;
  std::unique_ptr<actuator_test::ExternalDaqReader> m_external_daq;
  double m_sum_ext_torque = 0.0;
  double m_sum_ext_speed = 0.0;
  int m_ext_sample_count = 0;
  int m_series_ext_torque = -1;

  std::vector<JointInfo> m_joints;
  std::vector<double> m_steps_a;
  std::size_t m_step_index = 0;
  std::size_t m_forward_step_count = 0; ///< Steps before the mirrored return leg.
  bool m_running = false;
  std::size_t m_active_joint = 0;

  bool m_have_step_t0 = false;
  double m_step_t0_s = 0.0;
  double m_sum_current = 0.0;
  double m_sum_torque = 0.0;
  double m_peak_current = 0.0;
  double m_peak_torque = 0.0;
  int m_sample_count = 0;

  int m_series_cmd = -1;
  int m_series_current = -1;
  int m_series_torque = -1;
  int m_series_iv_drive_up = -1;
  int m_series_iv_drive_down = -1;
  int m_series_iv_ext_up = -1;
  int m_series_iv_ext_down = -1;
};

} // namespace actuator_test::gui
