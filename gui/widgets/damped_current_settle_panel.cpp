// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/damped_current_settle_panel.hpp"
#include "widgets/plot_panel.hpp"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QScrollBar>
#include <QSplitter>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace actuator_test::gui {

DampedCurrentSettleDialog::DampedCurrentSettleDialog(QWidget *parent)
    : QDialog(parent) {
  setWindowTitle(tr("Damped Current Settle"));
  setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                 Qt::WindowMinimizeButtonHint);
  setMinimumSize(900, 600);
  resize(1200, 800);

  auto *layout = new QVBoxLayout(this);
  auto *splitter = new QSplitter(Qt::Horizontal, this);
  layout->addWidget(splitter);
  auto *settings_panel = new QWidget();
  auto *settings_layout = new QVBoxLayout(settings_panel);
  settings_layout->setContentsMargins(0, 0, 0, 0);
  splitter->addWidget(settings_panel);
  auto *plots_panel = new QWidget();
  auto *plots_layout = new QVBoxLayout(plots_panel);
  plots_layout->setContentsMargins(0, 0, 0, 0);
  splitter->addWidget(plots_panel);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  auto *intro = new QLabel(tr(
      "<b>Damped current preconditioning.</b> Applies I(t) = A0 exp(-t/tau) "
      "sin(2 pi f t), then holds 0 A while the external torsiometer offset "
      "is measured. The shaft must be free to settle; do not lock the rotor."));
  intro->setWordWrap(true);
  settings_layout->addWidget(intro);

  auto *box = new QGroupBox(tr("Command configuration"));
  auto *form = new QFormLayout(box);
  m_joint_combo = new QComboBox();
  form->addRow(tr("Joint:"), m_joint_combo);
  m_amplitude_spin = new QDoubleSpinBox();
  m_amplitude_spin->setRange(0.05, 70.0);
  m_amplitude_spin->setDecimals(2);
  m_amplitude_spin->setValue(1.0);
  m_amplitude_spin->setSuffix(tr(" A"));
  form->addRow(tr("Initial amplitude (A0):"), m_amplitude_spin);
  m_decay_spin = new QDoubleSpinBox();
  m_decay_spin->setRange(0.05, 60.0);
  m_decay_spin->setDecimals(2);
  m_decay_spin->setValue(2.0);
  m_decay_spin->setSuffix(tr(" s"));
  form->addRow(tr("Decay constant (tau):"), m_decay_spin);
  m_frequency_spin = new QDoubleSpinBox();
  m_frequency_spin->setRange(0.05, 20.0);
  m_frequency_spin->setDecimals(2);
  m_frequency_spin->setValue(1.0);
  m_frequency_spin->setSuffix(tr(" Hz"));
  form->addRow(tr("Frequency:"), m_frequency_spin);
  m_duration_spin = new QDoubleSpinBox();
  m_duration_spin->setRange(0.1, 120.0);
  m_duration_spin->setDecimals(1);
  m_duration_spin->setValue(8.0);
  m_duration_spin->setSuffix(tr(" s"));
  form->addRow(tr("Excitation duration:"), m_duration_spin);
  m_settle_spin = new QDoubleSpinBox();
  m_settle_spin->setRange(0.1, 30.0);
  m_settle_spin->setDecimals(1);
  m_settle_spin->setValue(1.0);
  m_settle_spin->setSuffix(tr(" s"));
  form->addRow(tr("Zero-current measurement:"), m_settle_spin);
  m_confirm_check =
      new QCheckBox(tr("I confirm the shaft is free to move safely"));
  form->addRow(m_confirm_check);
  settings_layout->addWidget(box);

  auto *buttons = new QHBoxLayout();
  m_start_btn = new QPushButton(tr("Start settle"));
  m_stop_btn = new QPushButton(tr("STOP"));
  m_tare_btn = new QPushButton(tr("TARE"));
  m_stop_btn->setEnabled(false);
  m_tare_btn->setEnabled(false);
  buttons->addWidget(m_start_btn);
  buttons->addWidget(m_stop_btn);
  buttons->addWidget(m_tare_btn);
  settings_layout->addLayout(buttons);
  m_status_label = new QLabel(tr("Idle."));
  m_status_label->setWordWrap(true);
  settings_layout->addWidget(m_status_label);
  m_live_label = new QLabel(tr("Commanded: -- A   Actual: -- A"));
  settings_layout->addWidget(m_live_label);
  m_offset_label = new QLabel(tr("External raw offset: -- Nm   Std dev: -- Nm"));
  settings_layout->addWidget(m_offset_label);
  settings_layout->addStretch(1);
  m_current_chart = new StripChart(tr("Current (A)"));
  m_current_chart->setAxisTitles(tr("s"), tr("A"));
  m_current_chart->setMinimumHeight(220);
  m_current_chart->setPannable(true);
  m_current_chart->setMaxPoints(10000);
  m_current_chart->setSymmetric(true);
  m_series_commanded_current = m_current_chart->addSeries(
      tr("commanded"), QColor(90, 170, 250));
  m_series_actual_current = m_current_chart->addSeries(
      tr("actual"), QColor(250, 190, 60));
  plots_layout->addWidget(m_current_chart, 1);
  m_external_torque_chart = new StripChart(tr("External Raw Torque (Nm)"));
  m_external_torque_chart->setAxisTitles(tr("s"), tr("Nm"));
  m_external_torque_chart->setMinimumHeight(220);
  m_external_torque_chart->setPannable(true);
  m_external_torque_chart->setMaxPoints(10000);
  m_external_torque_chart->setSymmetric(true);
  m_series_external_raw_torque = m_external_torque_chart->addSeries(
      tr("torsiometer raw"), QColor(90, 210, 210));
  plots_layout->addWidget(m_external_torque_chart, 1);
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
  m_time_scrollbar->setPageStep(100);
  m_time_scrollbar->setEnabled(false);
  scrub_row->addWidget(m_time_scrollbar, 1);
  m_live_btn = new QPushButton(tr("Live"));
  m_live_btn->setCheckable(true);
  m_live_btn->setChecked(true);
  m_live_btn->setEnabled(false);
  scrub_row->addWidget(m_live_btn);
  plots_layout->addLayout(scrub_row);

  connect(m_start_btn, &QPushButton::clicked, this,
          &DampedCurrentSettleDialog::start);
  connect(m_stop_btn, &QPushButton::clicked, this,
          [this] { stop(tr("stopped by user")); });
  connect(m_tare_btn, &QPushButton::clicked, this,
          &DampedCurrentSettleDialog::tare);
  connect(m_confirm_check, &QCheckBox::toggled, this,
          [this](bool) { updateStartEnabled(); });
  connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) {
            updateAmplitudeLimit();
            updateStartEnabled();
          });
  connect(m_window_spin, &QDoubleSpinBox::valueChanged, this,
          [this](double seconds) {
            m_current_chart->setWindowSeconds(seconds);
            m_external_torque_chart->setWindowSeconds(seconds);
            m_time_scrollbar->setPageStep(
                std::max(1, static_cast<int>(std::round(seconds * 10.0))));
          });
  connect(m_time_scrollbar, &QScrollBar::valueChanged, this, [this](int value) {
    if (m_scrub_updating) {
      return;
    }
    m_live_btn->setChecked(false);
    const double end_s = value / 10.0;
    m_current_chart->setViewEnd(end_s);
    m_external_torque_chart->setViewEnd(end_s);
  });
  connect(m_live_btn, &QPushButton::toggled, this, [this](bool live) {
    if (!live) {
      return;
    }
    m_current_chart->followLatest();
    m_external_torque_chart->followLatest();
    m_scrub_updating = true;
    m_time_scrollbar->setValue(m_time_scrollbar->maximum());
    m_scrub_updating = false;
  });
  updateStartEnabled();
}

void DampedCurrentSettleDialog::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;
  const QString previous = m_joint_combo->currentText();
  m_joint_combo->blockSignals(true);
  m_joint_combo->clear();
  for (const auto &joint : m_joints) {
    m_joint_combo->addItem(QString::fromStdString(joint.name));
  }
  const int index = m_joint_combo->findText(previous);
  m_joint_combo->setCurrentIndex(index >= 0 ? index : 0);
  m_joint_combo->blockSignals(false);
  updateAmplitudeLimit();
  updateStartEnabled();
}

void DampedCurrentSettleDialog::setExternalDaqReader(
    std::shared_ptr<actuator_test::ExternalDaqReader> external_daq) {
  m_external_daq = std::move(external_daq);
}

std::size_t DampedCurrentSettleDialog::selectedJoint() const {
  return static_cast<std::size_t>(std::max(0, m_joint_combo->currentIndex()));
}

void DampedCurrentSettleDialog::updateAmplitudeLimit() {
  const std::size_t joint = selectedJoint();
  const double rated_current_a =
      joint < m_joints.size() ? m_joints[joint].rated_current_a : 0.0;
  m_amplitude_spin->setMaximum(std::max(0.05, rated_current_a));
}

void DampedCurrentSettleDialog::updateStartEnabled() {
  m_start_btn->setEnabled(!m_running && m_confirm_check->isChecked() &&
                          !m_joints.empty() &&
                          m_joint_combo->currentIndex() >= 0 &&
                          m_external_daq && m_external_daq->running());
}

void DampedCurrentSettleDialog::start() {
  if (!m_start_btn->isEnabled()) {
    return;
  }
  m_active_joint = selectedJoint();
  m_running = true;
  m_have_t0 = false;
  m_sum_torque_nm = 0.0;
  m_sum_sq_torque_nm = 0.0;
  m_torque_samples = 0;
  m_last_external_sample_t_s = -1.0;
  m_measured_offset_nm.reset();
  m_tare_btn->setEnabled(false);
  m_current_chart->clearAll();
  m_external_torque_chart->clearAll();
  m_current_chart->followLatest();
  m_external_torque_chart->followLatest();
  m_scrub_updating = true;
  m_time_scrollbar->setRange(0, 0);
  m_scrub_updating = false;
  m_live_btn->setChecked(true);
  m_offset_label->setText(tr("External raw offset: measuring..."));
  emit dampedCurrentRequested(m_active_joint, m_amplitude_spin->value(),
                              m_decay_spin->value(), m_frequency_spin->value(),
                              m_duration_spin->value(), false);
  m_status_label->setText(tr("Applying damped current for %1 s.")
                              .arg(m_duration_spin->value(), 0, 'f', 1));
  m_joint_combo->setEnabled(false);
  m_amplitude_spin->setEnabled(false);
  m_decay_spin->setEnabled(false);
  m_frequency_spin->setEnabled(false);
  m_duration_spin->setEnabled(false);
  m_settle_spin->setEnabled(false);
  m_confirm_check->setEnabled(false);
  m_start_btn->setEnabled(false);
  m_stop_btn->setEnabled(true);
}

void DampedCurrentSettleDialog::appendTelemetry(const TelemetryFrame &frame) {
  if (!m_running || m_active_joint >= frame.joints.size()) {
    return;
  }
  if (!m_have_t0) {
    m_t0_s = frame.t_s;
    m_have_t0 = true;
  }
  const double elapsed_s = frame.t_s - m_t0_s;
  const auto &joint = frame.joints[m_active_joint];
  const double command_a =
      elapsed_s < m_duration_spin->value()
          ? m_amplitude_spin->value() *
                std::exp(-elapsed_s / m_decay_spin->value()) *
                std::sin(2.0 * std::numbers::pi * m_frequency_spin->value() *
                         elapsed_s)
          : 0.0;
  m_live_label->setText(tr("Commanded: %1 A   Actual: %2 A")
                            .arg(command_a, 0, 'f', 3)
                            .arg(joint.current_a, 0, 'f', 3));
  m_current_chart->append(m_series_commanded_current, elapsed_s, command_a);
  m_current_chart->append(m_series_actual_current, elapsed_s, joint.current_a);
  const int max_scrub = static_cast<int>(std::round(elapsed_s * 10.0));
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
  if (joint.fault) {
    stop(tr("joint '%1' faulted").arg(QString::fromStdString(joint.name)));
    return;
  }
  bool have_new_external_sample = false;
  double external_raw_torque_nm = 0.0;
  if (m_external_daq && m_external_daq->running()) {
    const auto sample = m_external_daq->latest();
    if (sample.t_s > m_last_external_sample_t_s) {
      m_last_external_sample_t_s = sample.t_s;
      external_raw_torque_nm = sample.torque_nm;
      have_new_external_sample = true;
      m_external_torque_chart->append(m_series_external_raw_torque, elapsed_s,
                                      external_raw_torque_nm);
    }
  }
  if (elapsed_s < m_duration_spin->value()) {
    return;
  }
  m_status_label->setText(tr("Holding 0 A and measuring external raw torque "
                             "(%1 / %2 s).")
                              .arg(elapsed_s - m_duration_spin->value(), 0, 'f', 1)
                              .arg(m_settle_spin->value(), 0, 'f', 1));
  if (have_new_external_sample) {
    m_sum_torque_nm += external_raw_torque_nm;
    m_sum_sq_torque_nm += external_raw_torque_nm * external_raw_torque_nm;
    ++m_torque_samples;
  }
  if (elapsed_s >= m_duration_spin->value() + m_settle_spin->value()) {
    if (m_torque_samples > 0) {
      const double mean = m_sum_torque_nm / m_torque_samples;
      const double variance =
          std::max(0.0, m_sum_sq_torque_nm / m_torque_samples - mean * mean);
      m_measured_offset_nm = mean;
      m_offset_label->setText(
          tr("External raw offset: %1 Nm   Std dev: %2 Nm (%3 samples)")
              .arg(mean, 0, 'f', 4)
              .arg(std::sqrt(variance), 0, 'f', 4)
              .arg(m_torque_samples));
      m_tare_btn->setEnabled(m_external_daq && m_external_daq->running());
    } else {
      m_offset_label->setText(tr("External raw offset unavailable: no DAQ samples."));
    }
    stop(tr("settle and offset measurement complete"));
  }
}

void DampedCurrentSettleDialog::stop(const QString &reason) {
  if (!m_running) {
    return;
  }
  m_running = false;
  emit dampedCurrentRequested(m_active_joint, 0.0, 0.0, 0.0, 0.0, true);
  m_joint_combo->setEnabled(true);
  m_amplitude_spin->setEnabled(true);
  m_decay_spin->setEnabled(true);
  m_frequency_spin->setEnabled(true);
  m_duration_spin->setEnabled(true);
  m_settle_spin->setEnabled(true);
  m_confirm_check->setEnabled(true);
  m_stop_btn->setEnabled(false);
  m_status_label->setText(tr("Idle (%1).").arg(reason));
  updateStartEnabled();
}

void DampedCurrentSettleDialog::tare() {
  if (!m_measured_offset_nm || !m_external_daq ||
      !m_external_daq->tare(*m_measured_offset_nm)) {
    m_status_label->setText(
        tr("Tare failed: external torsiometer is not running."));
    return;
  }
  m_offset_label->setText(
      tr("External torsiometer tared by %1 Nm.")
          .arg(*m_measured_offset_nm, 0, 'f', 4));
  m_status_label->setText(tr("Tare applied to the external torsiometer."));
  m_tare_btn->setEnabled(false);
  m_measured_offset_nm.reset();
}

void DampedCurrentSettleDialog::closeEvent(QCloseEvent *event) {
  stop(tr("window closed"));
  event->accept();
}

} // namespace actuator_test::gui
