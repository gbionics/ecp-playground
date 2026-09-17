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
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSplitter>
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
  // QDialog's window type suppresses minimize/maximize decorations on most
  // window managers regardless of hints; force a normal top-level window so
  // the title bar buttons actually work.
  setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                 Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
  resize(1400, 850);

  auto *root_layout = new QVBoxLayout(this);
  auto *splitter = new QSplitter(Qt::Horizontal, this);
  root_layout->addWidget(splitter);

  // --- Left: settings, controls, results ----------------------------------
  auto *left_scroll = new QScrollArea();
  left_scroll->setWidgetResizable(true);
  left_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  left_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  left_scroll->setMinimumWidth(380);
  left_scroll->setMaximumWidth(460);
  auto *left_panel = new QWidget();
  auto *layout = new QVBoxLayout(left_panel);
  left_scroll->setWidget(left_panel);
  splitter->addWidget(left_scroll);

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

  m_invert_current_check = new QCheckBox(tr("Invert current polarity"));
  form->addRow(m_invert_current_check);
  auto *invert_current_hint = new QLabel(
      tr("Flips physical rotation direction. Does not fix a sign mismatch "
         "against the external sensor -- use \"Invert external sensor "
         "sign\" below for that."));
  invert_current_hint->setWordWrap(true);
  invert_current_hint->setStyleSheet(QStringLiteral("color: gray; font-size: 11px;"));
  form->addRow(invert_current_hint);

  m_confirm_check = new QCheckBox(
      tr("I confirm the rotor is mechanically locked / blocked"));
  form->addRow(m_confirm_check);

  layout->addWidget(config_box);

  // --- Optional external DAQ verification ---------------------------------
  auto *ext_box = new QGroupBox(tr("External DAQ verification (optional)"));
  auto *ext_form = new QFormLayout(ext_box);

  m_ext_daq_check = new QCheckBox(tr("Cross-check against a torque sensor / encoder"));
  ext_form->addRow(m_ext_daq_check);

  m_ext_invert_sign_check = new QCheckBox(tr("Invert external sensor sign"));
  ext_form->addRow(m_ext_invert_sign_check);
  auto *invert_sign_hint = new QLabel(
      tr("Aligns the sensor's own convention (e.g. CCW-positive) with "
         "positive commanded current."));
  invert_sign_hint->setWordWrap(true);
  invert_sign_hint->setStyleSheet(QStringLiteral("color: gray; font-size: 11px;"));
  ext_form->addRow(invert_sign_hint);

  m_ext_analog_edit = new QLineEdit(QStringLiteral("Dev1/ai28"));
  ext_form->addRow(tr("Torque sensor AI channel:"), m_ext_analog_edit);
  m_ext_digital_a_edit = new QLineEdit(QStringLiteral("Dev1/port0/line5"));
  ext_form->addRow(tr("Encoder quadrature A line:"), m_ext_digital_a_edit);
  m_ext_digital_b_edit = new QLineEdit(QStringLiteral("Dev1/port0/line6"));
  ext_form->addRow(tr("Encoder quadrature B line:"), m_ext_digital_b_edit);

  m_ext_daq_status_label = new QLabel();
  m_ext_daq_status_label->setWordWrap(true);
  ext_form->addRow(m_ext_daq_status_label);

  layout->addWidget(ext_box);

  updateExternalDaqAvailability();

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
  btn_row->addWidget(m_start_btn);
  btn_row->addWidget(m_stop_btn);
  layout->addLayout(btn_row);

  m_export_btn = new QPushButton(tr("Export Results CSV..."));
  m_export_btn->setEnabled(false);
  layout->addWidget(m_export_btn);

  m_status_label = new QLabel(tr("Idle."));
  m_status_label->setWordWrap(true);
  layout->addWidget(m_status_label);

  // --- Results table ---------------------------------------------------
  m_results_table = new QTableWidget(0, 8);
  m_results_table->setHorizontalHeaderLabels(
      {tr("Step"), tr("Commanded (A)"), tr("Avg Current (A)"),
       tr("Avg Torque (Nm)"), tr("Peak Current (A)"), tr("Peak Torque (Nm)"),
       tr("Ext. Torque (Nm)"), tr("Ext. Speed (RPM)")});
  m_results_table->horizontalHeader()->setStretchLastSection(true);
  m_results_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Interactive);
  m_results_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_results_table->setSelectionMode(QAbstractItemView::NoSelection);
  m_results_table->setMinimumHeight(160);
  m_results_table->setMinimumWidth(0);
  m_results_table->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  for (int col = 0; col < m_results_table->columnCount(); ++col) {
    m_results_table->setColumnWidth(col, 90);
  }
  layout->addWidget(m_results_table, 1);

  // --- Right: big live plots with a live-value readout above each ---------
  auto *right_scroll = new QScrollArea();
  right_scroll->setWidgetResizable(true);
  right_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  right_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  auto *right_panel = new QWidget();
  auto *right_layout = new QVBoxLayout(right_panel);
  right_scroll->setWidget(right_panel);
  splitter->addWidget(right_scroll);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  m_current_value_label = new QLabel(tr("Commanded: -- A   Actual: -- A"));
  m_current_value_label->setStyleSheet(
      QStringLiteral("font-size: 15px; font-weight: bold;"));
  right_layout->addWidget(m_current_value_label);

  m_current_chart = new StripChart(tr("Current (A)"));
  m_current_chart->setAxisTitles(tr("s"), tr("A"));
  m_current_chart->setMinimumHeight(260);
  m_current_chart->setPannable(true);
  m_current_chart->setMaxPoints(200000);
  m_series_cmd = m_current_chart->addSeries(tr("commanded"), QColor(90, 170, 250));
  m_series_current =
      m_current_chart->addSeries(tr("actual"), QColor(250, 190, 60));
  right_layout->addWidget(m_current_chart, 1);

  m_torque_value_label =
      new QLabel(tr("Drive torque: -- Nm   External sensor: -- Nm"));
  m_torque_value_label->setStyleSheet(
      QStringLiteral("font-size: 15px; font-weight: bold;"));
  right_layout->addWidget(m_torque_value_label);

  m_torque_chart = new StripChart(tr("Torque (Nm)"));
  m_torque_chart->setAxisTitles(tr("s"), tr("Nm"));
  m_torque_chart->setMinimumHeight(260);
  m_torque_chart->setPannable(true);
  m_torque_chart->setMaxPoints(200000);
  m_series_torque =
      m_torque_chart->addSeries(tr("actual (drive)"), QColor(120, 220, 140));
  m_series_ext_torque = m_torque_chart->addSeries(
      tr("actual (external sensor)"), QColor(220, 120, 220));
  right_layout->addWidget(m_torque_chart, 1);

  m_iv_chart = new StripChart(tr("Current vs Torque (per-step average)"));
  m_iv_chart->setAxisTitles(tr("A"), tr("Nm"));
  m_iv_chart->setXAxis(StripChart::XAxis::Value);
  m_iv_chart->setMinimumHeight(260);
  m_series_iv_drive_up =
      m_iv_chart->addSeries(tr("drive (up)"), QColor(120, 220, 140));
  m_series_iv_drive_down =
      m_iv_chart->addSeries(tr("drive (down)"), QColor(60, 130, 80));
  m_series_iv_ext_up =
      m_iv_chart->addSeries(tr("external (up)"), QColor(220, 120, 220));
  m_series_iv_ext_down =
      m_iv_chart->addSeries(tr("external (down)"), QColor(140, 60, 140));
  right_layout->addWidget(m_iv_chart, 1);

  // Both charts share the same time axis, so one scrollbar/window-size
  // control scrubs and rescales both together.
  auto *scrub_row = new QHBoxLayout();
  scrub_row->addWidget(new QLabel(tr("Time window:")));
  m_window_spin = new QDoubleSpinBox();
  m_window_spin->setRange(1.0, 300.0);
  m_window_spin->setDecimals(0);
  m_window_spin->setSuffix(tr(" s"));
  m_window_spin->setValue(10.0);
  scrub_row->addWidget(m_window_spin);
  scrub_row->addWidget(new QLabel(tr("Scroll through plot:")));
  m_time_scrollbar = new QScrollBar(Qt::Horizontal);
  m_time_scrollbar->setRange(0, 0);
  m_time_scrollbar->setPageStep(100); // 10 s window at 0.1 s resolution.
  m_time_scrollbar->setEnabled(false);
  scrub_row->addWidget(m_time_scrollbar, 1);
  m_live_btn = new QPushButton(tr("Live"));
  m_live_btn->setCheckable(true);
  m_live_btn->setChecked(true);
  m_live_btn->setEnabled(false);
  scrub_row->addWidget(m_live_btn);
  right_layout->addLayout(scrub_row);

  connect(m_window_spin, &QDoubleSpinBox::valueChanged, this,
          [this](double seconds) {
            m_current_chart->setWindowSeconds(seconds);
            m_torque_chart->setWindowSeconds(seconds);
            m_time_scrollbar->setPageStep(
                std::max(1, static_cast<int>(std::round(seconds * 10.0))));
          });
  connect(m_time_scrollbar, &QScrollBar::valueChanged, this, [this](int value) {
    if (m_scrub_updating) {
      return;
    }
    m_live_btn->setChecked(false);
    const double end_x = value / 10.0;
    m_current_chart->setViewEnd(end_x);
    m_torque_chart->setViewEnd(end_x);
  });
  connect(m_live_btn, &QPushButton::toggled, this, [this](bool live) {
    if (live) {
      m_current_chart->followLatest();
      m_torque_chart->followLatest();
      m_scrub_updating = true;
      m_time_scrollbar->setValue(m_time_scrollbar->maximum());
      m_scrub_updating = false;
    }
  });

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
  connect(m_ext_daq_check, &QCheckBox::toggled, this,
          [this](bool) { updateExternalDaqAvailability(); });

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

double LockedRotorTestDialog::appliedCurrentA(double requested_a) const {
  return m_invert_current_check->isChecked() ? -requested_a : requested_a;
}

void LockedRotorTestDialog::updateStartEnabled() {
  const bool ok = m_confirm_check->isChecked() && !m_joints.empty() &&
                  m_joint_combo->currentIndex() >= 0;
  m_start_btn->setEnabled(ok && !m_running);
}

void LockedRotorTestDialog::updateExternalDaqAvailability() {
  const bool supported = actuator_test::external_daq_supported();
  m_ext_daq_check->setEnabled(supported && !m_running);
  if (!supported) {
    m_ext_daq_check->setChecked(false);
    m_ext_daq_status_label->setText(
        tr("NI-DAQmx driver not available in this build; external "
           "verification disabled."));
  } else {
    m_ext_daq_status_label->setText(
        tr("NI-DAQmx available. Channels default to the session1 reference "
           "setup."));
  }
  const bool fields_enabled =
      supported && m_ext_daq_check->isChecked() && !m_running;
  m_ext_analog_edit->setEnabled(fields_enabled);
  m_ext_digital_a_edit->setEnabled(fields_enabled);
  m_ext_digital_b_edit->setEnabled(fields_enabled);
  m_ext_invert_sign_check->setEnabled(supported && !m_running);
}

void LockedRotorTestDialog::buildSteps() {
  m_steps_a.clear();
  const double start = m_start_spin->value();
  const double end = m_end_spin->value();
  const double step = std::max(0.05, std::fabs(m_step_spin->value()));
  if (std::fabs(end - start) < 1e-9) {
    m_steps_a.push_back(start);
    m_forward_step_count = m_steps_a.size();
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
  m_forward_step_count = m_steps_a.size();

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
  m_iv_chart->clearAll();
  m_current_chart->followLatest();
  m_torque_chart->followLatest();
  m_scrub_updating = true;
  m_time_scrollbar->setRange(0, 0);
  m_scrub_updating = false;
  m_live_btn->setChecked(true);
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
  m_invert_current_check->setEnabled(false);
  m_confirm_check->setEnabled(false);
  m_start_btn->setEnabled(false);
  m_stop_btn->setEnabled(true);
  m_ext_daq_check->setEnabled(false);
  m_ext_analog_edit->setEnabled(false);
  m_ext_digital_a_edit->setEnabled(false);
  m_ext_digital_b_edit->setEnabled(false);

  if (m_ext_daq_check->isChecked()) {
    actuator_test::ExternalDaqConfig ext_cfg;
    ext_cfg.analog_channel = m_ext_analog_edit->text().toStdString();
    ext_cfg.digital_line_a = m_ext_digital_a_edit->text().toStdString();
    ext_cfg.digital_line_b = m_ext_digital_b_edit->text().toStdString();
    ext_cfg.invert_torque_sign = m_ext_invert_sign_check->isChecked();
    m_external_daq =
        std::make_unique<actuator_test::ExternalDaqReader>(ext_cfg);
    std::string error;
    if (!m_external_daq->start(error)) {
      QMessageBox::warning(
          this, tr("External DAQ"),
          tr("Could not start external DAQ verification: %1\n\nContinuing "
             "the sweep without it.")
              .arg(QString::fromStdString(error)));
      m_external_daq.reset();
      m_ext_daq_status_label->setText(
          tr("External DAQ failed to start: %1").arg(QString::fromStdString(error)));
    } else {
      m_ext_daq_status_label->setText(
          tr("External DAQ verification active (%1, %2/%3).")
              .arg(m_ext_analog_edit->text(), m_ext_digital_a_edit->text(),
                   m_ext_digital_b_edit->text()));
    }
  }

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
  m_sum_ext_torque = 0.0;
  m_sum_ext_speed = 0.0;
  m_ext_sample_count = 0;

  const double target = appliedCurrentA(m_steps_a[index]);
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
  const double commanded = appliedCurrentA(m_steps_a[m_step_index]);

  m_current_chart->append(m_series_cmd, frame.t_s, commanded);
  m_current_chart->append(m_series_current, frame.t_s, jt.current_a);
  m_torque_chart->append(m_series_torque, frame.t_s, jt.torque_nm);

  const int max_scrub = static_cast<int>(std::round(frame.t_s * 10.0));
  m_scrub_updating = true;
  m_time_scrollbar->setRange(0, std::max(0, max_scrub));
  m_scrub_updating = false;
  m_time_scrollbar->setEnabled(max_scrub > 0);
  m_live_btn->setEnabled(max_scrub > 0);
  if (m_live_btn->isChecked()) {
    m_scrub_updating = true;
    m_time_scrollbar->setValue(max_scrub);
    m_scrub_updating = false;
  }

  m_current_value_label->setText(
      tr("Commanded: %1 A   Actual: %2 A")
          .arg(commanded, 0, 'f', 3)
          .arg(jt.current_a, 0, 'f', 3));
  m_torque_value_label->setText(
      tr("Drive torque: %1 Nm   External sensor: %2 Nm")
          .arg(jt.torque_nm, 0, 'f', 4)
          .arg(m_external_daq
                   ? QString::number(m_external_daq->latest().filtered_torque_nm, 'f', 4)
                   : QStringLiteral("--")));

  m_sum_current += jt.current_a;
  m_sum_torque += jt.torque_nm;
  m_peak_current = std::max(m_peak_current, std::fabs(jt.current_a));
  m_peak_torque = std::max(m_peak_torque, std::fabs(jt.torque_nm));
  ++m_sample_count;

  if (m_external_daq && m_external_daq->running()) {
    const auto ext = m_external_daq->latest();
    m_torque_chart->append(m_series_ext_torque, frame.t_s,
                           ext.filtered_torque_nm);
    m_sum_ext_torque += ext.filtered_torque_nm;
    m_sum_ext_speed += ext.speed_rpm;
    ++m_ext_sample_count;
  } else if (m_external_daq && !m_external_daq->running()) {
    // The acquisition thread died on its own (e.g. a DAQmx read error) --
    // surface why instead of silently leaving the external curve flat/absent.
    const QString reason = QString::fromStdString(m_external_daq->lastError());
    m_external_daq.reset();
    m_ext_daq_status_label->setText(
        tr("External DAQ stopped: %1")
            .arg(reason.isEmpty() ? tr("(no error reported)") : reason));
    QMessageBox::warning(
        this, tr("External DAQ"),
        tr("External DAQ verification stopped unexpectedly: %1\n\nThe sweep "
           "continues without it.")
            .arg(reason.isEmpty() ? tr("(no error reported)") : reason));
  }

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
  r.commanded_a = appliedCurrentA(m_steps_a[m_step_index]);
  r.avg_current_a =
      m_sample_count > 0 ? m_sum_current / m_sample_count : 0.0;
  r.avg_torque_nm = m_sample_count > 0 ? m_sum_torque / m_sample_count : 0.0;
  r.peak_current_a = m_peak_current;
  r.peak_torque_nm = m_peak_torque;
  r.ext_avg_torque_nm =
      m_ext_sample_count > 0 ? m_sum_ext_torque / m_ext_sample_count : 0.0;
  r.ext_avg_speed_rpm =
      m_ext_sample_count > 0 ? m_sum_ext_speed / m_ext_sample_count : 0.0;
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
  if (m_external_daq) {
    m_external_daq->stop();
    m_external_daq.reset();
  }

  m_joint_combo->setEnabled(true);
  m_start_spin->setEnabled(true);
  m_end_spin->setEnabled(true);
  m_step_spin->setEnabled(true);
  m_dwell_spin->setEnabled(true);
  m_return_sweep_check->setEnabled(true);
  m_invert_current_check->setEnabled(true);
  m_confirm_check->setEnabled(true);
  m_stop_btn->setEnabled(false);
  m_export_btn->setEnabled(m_results_table->rowCount() > 0);
  updateStartEnabled();
  updateExternalDaqAvailability();

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
  setCell(6, m_external_daq ? QString::number(r.ext_avg_torque_nm, 'f', 4)
                           : QStringLiteral("--"));
  setCell(7, m_external_daq ? QString::number(r.ext_avg_speed_rpm, 'f', 1)
                           : QStringLiteral("--"));
  m_results_table->scrollToBottom();

  // Route to the up-sweep or down-sweep (mirrored return leg) series so the
  // two directions can be told apart for hysteresis study.
  const bool ascending = m_step_index < m_forward_step_count;
  m_iv_chart->append(ascending ? m_series_iv_drive_up : m_series_iv_drive_down,
                     r.avg_current_a, r.avg_torque_nm);
  if (m_external_daq) {
    m_iv_chart->append(
        ascending ? m_series_iv_ext_up : m_series_iv_ext_down,
        r.avg_current_a, r.ext_avg_torque_nm);
  }
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
         "torque_nm,ext_avg_torque_nm,ext_avg_speed_rpm\n";
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
