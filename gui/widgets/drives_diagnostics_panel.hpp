// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Drive diagnostics panel showing drive status, temperatures, limits and
// fault information for all joints.

#pragma once

#include "core/controller_worker.hpp"

#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QTableWidget;
class QComboBox;
class QLabel;
QT_END_NAMESPACE

namespace actuator_test::gui {

/// Panel displaying drive-level diagnostics: temperatures, fault codes,
/// homing status, and limit information.
class DrivesDiagnosticsPanel : public QWidget {
  Q_OBJECT

public:
  explicit DrivesDiagnosticsPanel(QWidget *parent = nullptr);

  void setJoints(const std::vector<JointInfo> &joints);
  void updateTelemetry(const std::vector<JointTelemetry> &joints);

private slots:
  void onJointSelectionChanged(int index);

private:
  void rebuildDiagnosticsTable();

  QComboBox *m_joint_combo = nullptr;
  QTableWidget *m_diagnostics_table = nullptr;
  QLabel *m_status_label = nullptr;
  QLabel *m_homing_status_label = nullptr;
  std::vector<JointInfo> m_joints;
  std::vector<JointTelemetry> m_current_telemetry;
};

} // namespace actuator_test::gui
