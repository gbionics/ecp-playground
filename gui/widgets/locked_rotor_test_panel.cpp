// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/locked_rotor_test_panel.hpp"
#include "widgets/plot_panel.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace actuator_test::gui {

LockedRotorTestDialog::LockedRotorTestDialog(QWidget *parent)
    : QDialog(parent) {
  setWindowTitle(tr("Locked-Rotor Test"));
  resize(760, 620);

  auto *layout = new QVBoxLayout(this);

  auto *intro = new QLabel(
      tr("<b>Locked-rotor current sweep.</b> With the shaft mechanically "
         "blocked, this sends a programmable current setpoint sweep (no "
         "position/velocity feedback) to a single joint and records the "
         "current \u2192 torque response at each step."));
  intro->setWordWrap(true);
  layout->addWidget(intro);

  // --- Configuration -------------------------------------------------------
  auto *config_box = new QGroupBox(tr("Sweep configuration"));
  auto *form = new QFormLayout(config_box);

  m_joint_combo = new QComboBox();
  form->addRow(tr("Joint:"), m_joint_combo);

  m_start_spin = new QDoubleSpinBox();
  m_start_spin->setRange(-50.0, 50.0);
  m_start_spin->setDecimals(2);
  m_start_spin->setSuffix(tr(" A"));
  m_start_spin->setValue(0.0);
  form->addRow(tr("Start current:"), m_start_spin);

  m_end_spin = new QDoubleSpinBox();
  m_end_spin->setRange(-50.0, 50.0);
  m_end_spin->setDecimals(2);
  m_end_spin->setSuffix(tr(" A"));
  m_end_spin->setValue(5.0);
  form->addRow(tr("End current:"), m_end_spin);

  m_step_spin = new QDoubleSpinBox();
  m_step_spin->setRange(0.05, 20.0);
  m_step_spin->setDecimals(2);
  m_step_spin->setSuffix(tr(" A"));
  m_step_spin->setValue(0.5);
  form->addRow(tr("Step size:"), m_step_spin);

  m_dwell_spin = new QDoubleSpinBox();
  m_dwell_spin->setRange(0.2, 30.0);
  m_dwell_spin->setDecimals(1);
  m_dwell_spin->setSuffix(tr(" s"));
  m_dwell_spin->setValue(2.0);
  form->addRow(tr("Dwell per step:"), m_dwell_spin);

  m_return_sweep_check = new QCheckBox(
      tr("Return sweep back to start after reaching the end"));
  m_return_sweep_check->setChecked(true);
  form->addRow(m_return_sweep_check);

  m_confirm_check = new QCheckBox(
      tr("I confirm the rotor is mechanically locked / blocked"));
  form->addRow(m_confirm_check);

  layout->addWidget(config_box);

  // --- Controls ------------------------------------------------------------
  auto *btn_row = new QHBoxLayout();
  m_start_btn = new QPushButton(tr("Start Sweep"));
  m_stop_btn = new QPushButton(tr("STOP"));
  m_stop_btn->setEnabled(false);
  m_stop_btn->setMinimumHeight(36);
  m_stop_btn->setStyleSheet(
      QStringLiteral("QPushButton { background-color: #b3261e; color: white; "
                     "font-weight: bold; } "
                     "QPushButton:disabled { background-color: #5a5a5a; "
                     "color: #cccccc; }"));
  m_export_btn = new QPushButton(tr("Export Results CSV..."));
  m_export_btn->setEnabled(false);
  btn_row->addWidget(m_start_btn);
  btn_row->addWidget(m_stop_btn);
  btn_row->addStretch(1);
  btn_row->addWidget(m_export_btn);
  layout->addLayout(btn_row);

  m_status_label = new QLabel(tr("Idle."));
  layout->addWidget(m_status_label);

  // --- Live charts -----------------------------------------------------
  m_current_chart = new StripChart(tr("Current (A)"));
  m_current_chart->setAxisTitles(tr("s"), tr("A"));
  m_series_cmd = m_current_chart->addSeries(tr("commanded"), QColor(90, 170, 250));
  m_series_current =
      m_current_chart->addSeries(tr("actual"), QColor(250, 190, 60));
  layout->addWidget(m_current_chart, 1);

  m_torque_chart = new StripChart(tr("Torque (Nm)"));
  m_torque_chart->setAxisTitles(tr("s"), tr("Nm"));
  m_series_torque =
      m_torque_chart->addSeries(tr("actual"), QColor(120, 220, 140));
  layout->addWidget(m_torque_chart, 1);

  // --- Results table ---------------------------------------------------
  m_results_table = new QTableWidget(0, 6);
  m_results_table->setHorizontalHeaderLabels(
      {tr("Step"), tr("Commanded (A)"), tr("Avg Current (A)"),
       tr("Avg Torque (Nm)"), tr("Peak Current (A)"), tr("Peak Torque (Nm)")});
  m_results_table->horizontalHeader()->setStretchLastSection(true);
  m_results_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_results_table->setSelectionMode(QAbstractItemView::NoSelection);
  m_results_table->setMaximumHeight(180);
  layout->addWidget(m_results_table);

  connect(m_start_btn, &QPushButton::clicked, this,
          &LockedRotorTestDialog::onStartClicked);
  connect(m_stop_btn, &QPushButton::clicked, this,
          &LockedRotorTestDialog::onStopClicked);
  connect(m_export_btn, &QPushButton::clicked, this,
          &LockedRotorTestDialog::exportResultsCsv);
  connect(m_confirm_check, &QCheckBox::toggled, this,
          [this](bool) { updateStartEnabled(); });
  connect(m_joint_combo, &QComboBox::currentIndexChanged, this,
          [this](int) { updateStartEnabled(); });

  updateStartEnabled();
}

void LockedRotorTestDialog::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;
  const QString previous = m_joint_combo->currentText();
  m_joint_combo->blockSignals(true);
  m_joint_combo->clear();
  for (const auto &j : joints) {
    m_joint_combo->addItem(QString::fromStdString(j.name));
  }
  const int idx = m_joint_combo->findText(previous);
  m_joint_combo->setCurrentIndex(idx >= 0 ? idx : 0);
  m_joint_combo->blockSignals(false);
  updateStartEnabled();
}

std::size_t LockedRotorTestDialog::selectedJoint() const {
  return static_cast<std::size_t>(std::max(0, m_joint_combo->currentIndex()));
}

void LockedRotorTestDialog::updateStartEnabled() {
  const bool ok = m_confirm_check->isChecked() && !m_joints.empty() &&
                  m_joint_combo->currentIndex() >= 0;
  m_start_btn->setEnabled(ok && !m_running);
}

void LockedRotorTestDialog::buildSteps() {
  m_steps_a.clear();
  const double start = m_start_spin->value();
  const double end = m_end_spin->value();
  const double step = std::max(0.05, std::fabs(m_step_spin->value()));
  if (std::fabs(end - start) < 1e-9) {
    m_steps_a.push_back(start);
    return;
  }
  const double dir = (end > start) ? step : -step;
  for (double v = start;
       (dir > 0.0) ? (v <= end + 1e-9) : (v >= end - 1e-9); v += dir) {
    m_steps_a.push_back(v);
  }
  // Guarantee the requested end point is included even if it doesn't land
  // exactly on a step boundary.
  if (m_steps_a.empty() || std::fabs(m_steps_a.back() - end) > 1e-6) {
    m_steps_a.push_back(end);
  }

  // Optionally mirror the ramp back down to the start value (e.g. 0) so the
  // sweep doesn't leave the joint sitting at the peak current. Build the
  // mirrored part in a separate vector first: appending to m_steps_a while
  // iterating its own reverse_iterator would invalidate the iterator on
  // reallocation (undefined behaviour / crash).
  if (m_return_sweep_check->isChecked() && m_steps_a.size() > 1) {
    const std::vector<double> mirrored(m_steps_a.rbegin() + 1,
                                       m_steps_a.rend());
    m_steps_a.insert(m_steps_a.end(), mirrored.begin(), mirrored.end());
  }
}

void LockedRotorTestDialog::onStartClicked() {
  if (m_running || m_joints.empty()) {
    return;
  }
  buildSteps();
  if (m_steps_a.empty()) {
    return;
  }

  m_results_table->setRowCount(0);
  m_current_chart->clearAll();
  m_torque_chart->clearAll();
  m_export_btn->setEnabled(false);

  m_active_joint = selectedJoint();
  m_running = true;
  m_step_index = 0;

  m_joint_combo->setEnabled(false);
  m_start_spin->setEnabled(false);
  m_end_spin->setEnabled(false);
  m_step_spin->setEnabled(false);
  m_dwell_spin->setEnabled(false);
  m_return_sweep_check->setEnabled(false);
  m_confirm_check->setEnabled(false);
  m_start_btn->setEnabled(false);
  m_stop_btn->setEnabled(true);

  beginStep(0);
}

void LockedRotorTestDialog::beginStep(std::size_t index) {
  m_step_index = index;
  m_have_step_t0 = false;
  m_sum_current = 0.0;
  m_sum_torque = 0.0;
  m_peak_current = 0.0;
  m_peak_torque = 0.0;
  m_sample_count = 0;

  const double target = m_steps_a[index];
  emit currentSetpointRequested(m_active_joint, target, /*release=*/false);
  m_status_label->setText(
      tr("Step %1/%2: commanding %3 A, dwelling %4 s...")
          .arg(index + 1)
          .arg(m_steps_a.size())
          .arg(target, 0, 'f', 2)
          .arg(m_dwell_spin->value(), 0, 'f', 1));
}

void LockedRotorTestDialog::appendTelemetry(const TelemetryFrame &frame) {
  if (!m_running) {
    return;
  }
  if (m_active_joint >= frame.joints.size()) {
    stopTest(tr("joint disappeared from telemetry"));
    return;
  }
  if (!m_have_step_t0) {
    m_step_t0_s = frame.t_s;
    m_have_step_t0 = true;
  }

  const JointTelemetry &jt = frame.joints[m_active_joint];
  const double commanded = m_steps_a[m_step_index];

  m_current_chart->append(m_series_cmd, frame.t_s, commanded);
  m_current_chart->append(m_series_current, frame.t_s, jt.current_a);
  m_torque_chart->append(m_series_torque, frame.t_s, jt.torque_nm);

  m_sum_current += jt.current_a;
  m_sum_torque += jt.torque_nm;
  m_peak_current = std::max(m_peak_current, std::fabs(jt.current_a));
  m_peak_torque = std::max(m_peak_torque, std::fabs(jt.torque_nm));
  ++m_sample_count;

  if (jt.fault) {
    stopTest(tr("joint '%1' faulted").arg(QString::fromStdString(jt.name)));
    return;
  }

  const double elapsed = frame.t_s - m_step_t0_s;
  if (elapsed >= m_dwell_spin->value()) {
    finishStep();
  }
}

void LockedRotorTestDialog::finishStep() {
  StepResult r;
  r.commanded_a = m_steps_a[m_step_index];
  r.avg_current_a =
      m_sample_count > 0 ? m_sum_current / m_sample_count : 0.0;
  r.avg_torque_nm = m_sample_count > 0 ? m_sum_torque / m_sample_count : 0.0;
  r.peak_current_a = m_peak_current;
  r.peak_torque_nm = m_peak_torque;
  appendResultRow(static_cast<int>(m_step_index) + 1, r);

  const std::size_t next = m_step_index + 1;
  if (next >= m_steps_a.size()) {
    stopTest(tr("sweep complete"));
    return;
  }
  beginStep(next);
}

void LockedRotorTestDialog::onStopClicked() { stopTest(tr("stopped by user")); }

void LockedRotorTestDialog::stopTest(const QString &reason) {
  if (!m_running) {
    return;
  }
  m_running = false;
  emit currentSetpointRequested(m_active_joint, 0.0, /*release=*/true);

  m_joint_combo->setEnabled(true);
  m_start_spin->setEnabled(true);
  m_end_spin->setEnabled(true);
  m_step_spin->setEnabled(true);
  m_dwell_spin->setEnabled(true);
  m_return_sweep_check->setEnabled(true);
  m_confirm_check->setEnabled(true);
  m_stop_btn->setEnabled(false);
  m_export_btn->setEnabled(m_results_table->rowCount() > 0);
  updateStartEnabled();

  m_status_label->setText(tr("Idle (%1).").arg(reason));
}

void LockedRotorTestDialog::appendResultRow(int step_number,
                                            const StepResult &r) {
  const int row = m_results_table->rowCount();
  m_results_table->insertRow(row);
  auto setCell = [this, row](int col, const QString &text) {
    m_results_table->setItem(row, col, new QTableWidgetItem(text));
  };
  setCell(0, QString::number(step_number));
  setCell(1, QString::number(r.commanded_a, 'f', 3));
  setCell(2, QString::number(r.avg_current_a, 'f', 3));
  setCell(3, QString::number(r.avg_torque_nm, 'f', 4));
  setCell(4, QString::number(r.peak_current_a, 'f', 3));
  setCell(5, QString::number(r.peak_torque_nm, 'f', 4));
  m_results_table->scrollToBottom();
}

void LockedRotorTestDialog::exportResultsCsv() {
  if (m_results_table->rowCount() == 0) {
    return;
  }
  const QString suggested =
      QStringLiteral("locked-rotor-%1.csv")
          .arg(QDateTime::currentDateTime().toString(
              QStringLiteral("yyyyMMdd-HHmmss")));
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export locked-rotor results"), suggested,
      tr("CSV file (*.csv)"));
  if (path.isEmpty()) {
    return;
  }
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    QMessageBox::warning(this, tr("Export Results"),
                         tr("Could not write to %1").arg(path));
    return;
  }
  QTextStream out(&file);
  out << "step,commanded_a,avg_current_a,avg_torque_nm,peak_current_a,peak_"
         "torque_nm\n";
  for (int row = 0; row < m_results_table->rowCount(); ++row) {
    for (int col = 0; col < m_results_table->columnCount(); ++col) {
      if (col > 0) {
        out << ',';
      }
      out << m_results_table->item(row, col)->text();
    }
    out << '\n';
  }
}

} // namespace actuator_test::gui
