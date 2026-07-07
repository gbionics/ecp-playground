// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "drives_diagnostics_panel.hpp"

#include <QComboBox>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QString>
#include <QTableWidget>
#include <QVBoxLayout>

namespace actuator_test::gui {

DrivesDiagnosticsPanel::DrivesDiagnosticsPanel(QWidget *parent)
    : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);

  // Joint selector
  auto *selector_layout = new QHBoxLayout();
  selector_layout->addWidget(new QLabel(tr("Joint:")));
  m_joint_combo = new QComboBox();
  connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, &DrivesDiagnosticsPanel::onJointSelectionChanged);
  selector_layout->addWidget(m_joint_combo);
  selector_layout->addStretch();
  layout->addLayout(selector_layout);

  // Status labels
  m_status_label = new QLabel(tr("Status: --"));
  m_homing_status_label = new QLabel(tr("Homing: Not Completed"));
  layout->addWidget(m_status_label);
  layout->addWidget(m_homing_status_label);

  // Diagnostics table
  m_diagnostics_table = new QTableWidget(0, 2);
  m_diagnostics_table->setHorizontalHeaderLabels(
      {tr("Parameter"), tr("Value")});
  m_diagnostics_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  m_diagnostics_table->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::Stretch);
  m_diagnostics_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  layout->addWidget(m_diagnostics_table);

  setLayout(layout);
}

void DrivesDiagnosticsPanel::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;
  m_joint_combo->clear();
  for (const auto &j : joints) {
    m_joint_combo->addItem(QString::fromStdString(j.name));
  }
}

void DrivesDiagnosticsPanel::updateTelemetry(
    const std::vector<JointTelemetry> &joints) {
  if (joints.empty())
    return;

  m_current_telemetry = joints;

  int joint_idx = m_joint_combo->currentIndex();
  if (joint_idx < 0 || joint_idx >= static_cast<int>(joints.size()))
    return;

  const auto &telem = joints[joint_idx];

  // Update status labels
  QString status_text = QString(tr("Status Code: 0x%1 | Error Code: 0x%2"))
                            .arg(telem.status, 4, 16, QChar('0'))
                            .arg(telem.error_code, 4, 16, QChar('0'));
  m_status_label->setText(status_text);

  QString homing_text =
      telem.homed ? tr("Homing: Completed") : tr("Homing: Not Completed");
  if (telem.homing_active)
    homing_text = tr("Homing: In Progress");
  m_homing_status_label->setText(homing_text);

  // Update diagnostics table
  rebuildDiagnosticsTable();
}

void DrivesDiagnosticsPanel::onJointSelectionChanged(int index) {
  if (index >= 0 && index < static_cast<int>(m_current_telemetry.size())) {
    updateTelemetry(m_current_telemetry);
  }
}

void DrivesDiagnosticsPanel::rebuildDiagnosticsTable() {
  int joint_idx = m_joint_combo->currentIndex();
  if (joint_idx < 0 ||
      joint_idx >= static_cast<int>(m_current_telemetry.size()))
    return;

  const auto &telem = m_current_telemetry[joint_idx];
  m_diagnostics_table->setRowCount(0);

  auto addRow = [this](const QString &param, const QString &value) {
    int row = m_diagnostics_table->rowCount();
    m_diagnostics_table->insertRow(row);
    auto *param_item = new QTableWidgetItem(param);
    auto *value_item = new QTableWidgetItem(value);
    m_diagnostics_table->setItem(row, 0, param_item);
    m_diagnostics_table->setItem(row, 1, value_item);
  };

  addRow(tr("Position"), QString::number(telem.actual_deg, 'f', 2) + " deg");
  addRow(tr("Reference"),
         QString::number(telem.reference_deg, 'f', 2) + " deg");
  addRow(tr("Following Error"),
         QString::number(telem.following_error_deg, 'f', 3) + " deg");
  addRow(tr("Velocity"),
         QString::number(telem.velocity_deg_s, 'f', 2) + " deg/s");
  addRow(tr("Torque"), QString::number(telem.torque_percent, 'f', 1) + "%");
  addRow(tr("Power"), QString::number(telem.power_watts, 'f', 1) + " W");
  addRow(tr("Motor Temperature"),
         telem.motor_temp_c >= 0 ? QString::number(telem.motor_temp_c) + " °C"
                                 : tr("--"));
  addRow(tr("Drive Temperature"),
         telem.drive_temp_c >= 0 ? QString::number(telem.drive_temp_c) + " °C"
                                 : tr("--"));
  addRow(tr("Soft Limits"),
         QString::number(telem.soft_min_limit_deg, 'f', 1) + " to " +
             QString::number(telem.soft_max_limit_deg, 'f', 1) + " deg");
  addRow(tr("Hard Limits"),
         QString::number(telem.min_limit_deg, 'f', 1) + " to " +
             QString::number(telem.max_limit_deg, 'f', 1) + " deg");
  addRow(tr("Limit Violation"), telem.limit_violation ? tr("YES") : tr("No"));
  addRow(tr("Hard Limit Violation"),
         telem.hard_limit_violation ? tr("YES") : tr("No"));
  addRow(tr("Fault"), telem.fault ? tr("YES") : tr("No"));
  addRow(tr("Operation Mode"),
         QString::fromStdString(m_joints.size() > joint_idx
                                    ? m_joints[joint_idx].operation_mode_name
                                    : "Unknown"));
}

} // namespace actuator_test::gui
