// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Simplified limits panel with only two limit-apply modes:
// 1) Auto capture via backdrive
// 2) Manual min/max override

#pragma once

#include "core/controller_worker.hpp"

#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
class QLabel;
QT_END_NAMESPACE

namespace actuator_test::gui {

/// Limits panel providing two clear workflows:
/// - Auto capture: backdrive to min/max
/// - Manual override: type min/max and apply
class EnhancedLimitsPanel : public QWidget {
  Q_OBJECT

public:
  explicit EnhancedLimitsPanel(QWidget *parent = nullptr);

  void setJoints(const std::vector<JointInfo> &joints);
  void setState(ControllerState state);
  void updateLiveLimits(const std::vector<JointInfo> &joints,
                        const std::vector<JointTelemetry> &telemetry);
  std::size_t currentJoint() const;

signals:
  void captureToggled(bool start);
  void setLimitsRequested(std::size_t joint, double min_deg, double max_deg);
  void resetLimitsRequested(std::size_t joint);

private slots:
  void onJointChanged(int index);
  void onApplyLimitsClicked();

private:
  QComboBox *m_joint_combo = nullptr;

  // Auto + manual controls
  QPushButton *m_capture_btn = nullptr;
  QPushButton *m_apply_limits_btn = nullptr;
  QPushButton *m_reset_btn = nullptr;
  QDoubleSpinBox *m_hard_min_spin = nullptr;
  QDoubleSpinBox *m_hard_max_spin = nullptr;
  QLabel *m_auto_info_label = nullptr;
  QLabel *m_manual_info_label = nullptr;

  bool m_capturing = false; ///< True while a backdrive capture is running.

  std::vector<JointInfo> m_joints;
  std::vector<JointTelemetry> m_telemetry;
};

} // namespace actuator_test::gui
