// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "actuator_test/external_daq.hpp"
#include "core/controller_worker.hpp"
#include "core/telemetry.hpp"

#include <QDialog>
#include <memory>
#include <optional>
#include <vector>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QPushButton;
class QScrollBar;
QT_END_NAMESPACE

namespace actuator_test::gui {

class StripChart;

class DampedCurrentSettleDialog : public QDialog {
  Q_OBJECT

public:
  explicit DampedCurrentSettleDialog(QWidget *parent = nullptr);
  void setJoints(const std::vector<JointInfo> &joints);
  void setExternalDaqReader(
      std::shared_ptr<actuator_test::ExternalDaqReader> external_daq);
  void appendTelemetry(const TelemetryFrame &frame);

signals:
  void dampedCurrentRequested(std::size_t joint, double amplitude_a,
                              double decay_time_s, double frequency_hz,
                              double duration_s, bool release);

protected:
  void closeEvent(QCloseEvent *event) override;

private:
  void updateAmplitudeLimit();
  void start();
  void stop(const QString &reason);
  void tare();
  void updateStartEnabled();
  std::size_t selectedJoint() const;

  std::vector<JointInfo> m_joints;
  std::shared_ptr<actuator_test::ExternalDaqReader> m_external_daq;
  QComboBox *m_joint_combo = nullptr;
  QDoubleSpinBox *m_amplitude_spin = nullptr;
  QDoubleSpinBox *m_decay_spin = nullptr;
  QDoubleSpinBox *m_frequency_spin = nullptr;
  QDoubleSpinBox *m_duration_spin = nullptr;
  QDoubleSpinBox *m_settle_spin = nullptr;
  QCheckBox *m_confirm_check = nullptr;
  QPushButton *m_start_btn = nullptr;
  QPushButton *m_stop_btn = nullptr;
  QPushButton *m_tare_btn = nullptr;
  QLabel *m_status_label = nullptr;
  QLabel *m_live_label = nullptr;
  QLabel *m_offset_label = nullptr;
  StripChart *m_current_chart = nullptr;
  StripChart *m_external_torque_chart = nullptr;
  QDoubleSpinBox *m_window_spin = nullptr;
  QScrollBar *m_time_scrollbar = nullptr;
  QPushButton *m_live_btn = nullptr;
  int m_series_commanded_current = -1;
  int m_series_actual_current = -1;
  int m_series_external_raw_torque = -1;
  bool m_scrub_updating = false;
  bool m_running = false;
  bool m_have_t0 = false;
  double m_t0_s = 0.0;
  std::size_t m_active_joint = 0;
  double m_sum_torque_nm = 0.0;
  double m_sum_sq_torque_nm = 0.0;
  int m_torque_samples = 0;
  double m_last_external_sample_t_s = -1.0;
  std::optional<double> m_measured_offset_nm;
};

} // namespace actuator_test::gui
