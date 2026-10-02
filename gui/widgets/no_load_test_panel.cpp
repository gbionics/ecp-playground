// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
#include "no_load_test_panel.hpp"
#include "plot_panel.hpp"

#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTextStream>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>

namespace actuator_test::gui {
namespace {
constexpr double freshness_s = 0.25;
constexpr double torque_mean_period_s = 0.5;
constexpr std::size_t torque_mean_samples = 100;
bool validSample(const ExternalDaqReader::Sample &s) {
  return std::isfinite(s.t_s) && s.t_s > 0 && std::isfinite(s.torque_nm) &&
         std::isfinite(s.filtered_torque_nm) && std::isfinite(s.speed_rpm);
}
} // namespace

NoLoadTestDialog::NoLoadTestDialog(QWidget *parent, QString result_root)
    : QDialog(parent), m_result_root(std::move(result_root)) {
  setWindowTitle(tr("No-load speed test"));
  resize(1250, 760);
  m_clock.start();
  auto *layout = new QVBoxLayout(this);
  auto *instructions = new QLabel(tr(
      "DUT: PVT speed hold with temporary KP/KD. KP>0 tracks a position "
      "reference generated from speed; KP=0 uses velocity feedback only. "
      "Actual speed is "
      "verified within tolerance. Control the Yaskawa externally; this dialog "
      "does not command it.\nDAQ is mandatory. Torque must remain in the "
      "symmetric no-load band and both speeds within tolerance before "
      "capture.\n"
      "Completion does NOT stop rotation. Press Stop to release the DUT."));
  instructions->setWordWrap(true);
  layout->addWidget(instructions);
  m_settings = new QGroupBox(tr("Settings"));
  auto *form = new QFormLayout(m_settings);
  m_joint = new QComboBox;
  form->addRow(tr("DUT joint"), m_joint);
  auto spin = [form](const QString &label, double lo, double hi, double value,
                     int decimals) {
    auto *s = new QDoubleSpinBox;
    s->setRange(lo, hi);
    s->setDecimals(decimals);
    s->setValue(value);
    form->addRow(label, s);
    return s;
  };
  m_rpm = spin(tr("Target speed [rpm]"), -10000, 10000, 10, 3);
  m_ramp = spin(tr("Speed ramp [s]"), 0.01, 60, 1, 2);
  m_kp = new QSpinBox;
  m_kd = new QSpinBox;
  for (auto *gain : {m_kp, m_kd}) {
    gain->setRange(0, std::numeric_limits<int32_t>::max());
    gain->setToolTip(
        tr("Raw PVT drive gain, not SI units. Temporary for this "
           "test; the TOML is not modified. Stop before editing."));
    connect(gain, &QSpinBox::valueChanged, this, [this] { updateControls(); });
  }
  form->addRow(tr("PVT KP"), m_kp);
  form->addRow(tr("PVT KD"), m_kd);
  m_kp->setToolTip(m_kp->toolTip() +
                   tr("\nKP>0 integrates speed into a position reference. The "
                      "test stops before 32-bit position-count rollover; KP=0 "
                      "does not accumulate a position reference."));
  m_band = spin(tr("No-load torque band [± Nm]"), 0.001, 10000, 0.5, 3);
  m_tolerance = spin(tr("Speed tolerance [± rpm]"), 0.001, 1000, 1, 3);
  m_dwell = spin(tr("Continuous in-band dwell [s]"), 0.1, 600, 2, 2);
  m_continuous = new QCheckBox(tr("Continuous rotation - no travel limits"));
  m_continuous->setChecked(false);
  form->addRow(m_continuous);
  m_host = new QLineEdit(QString::fromStdString(PowerSupplyConfig{}.host));
  form->addRow(tr("Required PSU host (LAN)"), m_host);
  m_period = spin(tr("PSU sample period [s]"), 0.0001, 10, 0.01, 4);
  m_duration =
      spin(tr("Common finite acquisition duration [s]"), 0.5, 3600, 10, 2);

  auto *plots = new QVBoxLayout;
  m_speed_chart = new StripChart(tr("Speed (rpm)"));
  m_speed_chart->setAxisTitles(tr("s"), tr("rpm"));
  m_series_ref_speed =
      m_speed_chart->addSeries(tr("reference"), QColor(80, 160, 255));
  m_series_drive_speed =
      m_speed_chart->addSeries(tr("actual (drive)"), QColor(120, 220, 140));
  m_series_daq_speed = m_speed_chart->addSeries(tr("actual (torsiometer)"),
                                                QColor(240, 180, 60));
  m_torque_chart = new StripChart(tr("Torque - torsiometer raw (Nm)"));
  m_torque_chart->setAxisTitles(tr("s"), tr("Nm"));
  m_series_torque =
      m_torque_chart->addSeries(tr("torque"), QColor(120, 220, 140));
  m_series_band_hi =
      m_torque_chart->addSeries(tr("+band"), QColor(200, 200, 200));
  m_series_band_lo =
      m_torque_chart->addSeries(tr("-band"), QColor(200, 200, 200));
  for (auto *chart : {m_speed_chart, m_torque_chart}) {
    chart->setWindowSeconds(20.0);
    chart->setMinimumHeight(180);
    plots->addWidget(chart, 1);
  }
  m_power_chart =
      new StripChart(tr("DC supply power after acquisition (W, ELOG time)"));
  m_power_chart->setAxisTitles(tr("ELOG-relative s"), tr("W"));
  m_power_chart->setPannable(true);
  m_power_chart->setMaxPoints(1000000);
  m_power_chart->setMinimumHeight(180);
  m_series_power = m_power_chart->addSeries(tr("V x I"), QColor(240, 90, 90));
  plots->addWidget(m_power_chart, 1);
  m_power_label = new QLabel(tr("Power: available when acquisition ends."));
  m_power_label->setTextInteractionFlags(Qt::TextSelectableByMouse);
  plots->addWidget(m_power_label);

  // Buttons and status stay outside m_settings: the settings group is disabled
  // while busy, but Stop must remain usable.
  auto *left = new QVBoxLayout;
  left->addWidget(m_settings);
  m_live = new QLabel(tr("Waiting for fresh external DAQ samples."));
  m_status = new QLabel(tr("Idle"));
  m_saved = new QLabel;
  for (auto *label : {m_live, m_status, m_saved}) {
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    left->addWidget(label);
  }
  auto *buttons = new QGridLayout;
  m_start = new QPushButton(tr("Start speed / arm"));
  m_rearm = new QPushButton(tr("Rearm acquisition"));
  m_stop = new QPushButton(tr("Stop"));
  m_export = new QPushButton(tr("Export CSV copies..."));
  buttons->addWidget(m_start, 0, 0);
  buttons->addWidget(m_stop, 0, 1);
  buttons->addWidget(m_rearm, 1, 0);
  buttons->addWidget(m_export, 1, 1);
  left->addLayout(buttons);
  left->addStretch(1);

  auto *body = new QHBoxLayout;
  body->addLayout(left, 2);
  body->addLayout(plots, 3);
  layout->addLayout(body, 1);
  connect(m_start, &QPushButton::clicked, this, &NoLoadTestDialog::start);
  connect(m_stop, &QPushButton::clicked, this,
          [this] { stopTest(tr("Stopped by user")); });
  connect(m_rearm, &QPushButton::clicked, this, &NoLoadTestDialog::rearm);
  connect(m_export, &QPushButton::clicked, this,
          &NoLoadTestDialog::exportResults);
  connect(m_rpm, &QDoubleSpinBox::valueChanged, this,
          [this] { updateControls(); });
  connect(m_host, &QLineEdit::textChanged, this, [this] { updateControls(); });
  connect(m_joint, &QComboBox::currentIndexChanged, this,
          [this] { loadJointGains(); });
  auto *timer = new QTimer(this);
  timer->setInterval(25);
  connect(timer, &QTimer::timeout, this, &NoLoadTestDialog::tick);
  timer->start();
  updateControls();
}

void NoLoadTestDialog::setJoints(const std::vector<JointInfo> &joints) {
  if (isBusy())
    stopTest(tr("Joint enumeration changed"));
  m_joints = joints;
  const QSignalBlocker blocker(m_joint);
  m_joint->clear();
  for (const auto &joint : joints)
    m_joint->addItem(QString::fromStdString(joint.name));
  loadJointGains();
}

void NoLoadTestDialog::loadJointGains() {
  const int index = m_joint->currentIndex();
  const bool valid =
      index >= 0 && static_cast<std::size_t>(index) < m_joints.size();
  const QSignalBlocker kp_blocker(m_kp), kd_blocker(m_kd);
  m_kp->setValue(valid ? m_joints[index].pvt_kp : 0);
  m_kd->setValue(valid ? m_joints[index].pvt_kd : 0);
  if (valid && (m_joints[index].pvt_kp < 0 || m_joints[index].pvt_kd < 0))
    m_status->setText(
        tr("Configured PVT gains contain negative values. Enter non-negative "
           "test gains; at least one must be positive."));
  updateControls();
}

void NoLoadTestDialog::setExternalDaqReader(
    std::shared_ptr<ExternalDaqReader> reader) {
  if (isBusy())
    stopTest(tr("DAQ reader changed"));
  m_daq = std::move(reader);
  m_last_daq_t = -1;
  m_last_fresh_wall = -1;
  resetTorqueMean();
  updateControls();
}

void NoLoadTestDialog::resetTorqueMean() {
  m_torque_mean_start = -1;
  m_torque_sum = 0;
  m_torque_window.clear();
  m_torque_mean_text = tr("mean (last 100 samples): --");
}

void NoLoadTestDialog::updateTorqueReadout(
    const ExternalDaqReader::Sample &sample, double wall) {
  if (m_torque_mean_start < 0)
    m_torque_mean_start = wall;
  m_torque_sum += sample.torque_nm;
  m_torque_window.push_back(sample.torque_nm);
  if (m_torque_window.size() > torque_mean_samples) {
    m_torque_sum -= m_torque_window.front();
    m_torque_window.pop_front();
  }
  if (wall - m_torque_mean_start >= torque_mean_period_s) {
    m_torque_mean_text =
        tr("mean (%1/100 samples): %2 Nm")
            .arg(m_torque_window.size())
            .arg(m_torque_sum / static_cast<double>(m_torque_window.size()), 0,
                 'f', 3);
    m_torque_mean_start = wall;
  }
  m_live->setText(tr("External raw torque: %1 Nm | %2 | speed: %3 rpm")
                      .arg(sample.torque_nm, 0, 'f', 3)
                      .arg(m_torque_mean_text)
                      .arg(sample.speed_rpm, 0, 'f', 2));
}

void NoLoadTestDialog::setAvailable(bool available) {
  m_available = available;
  updateControls();
}

void NoLoadTestDialog::updateControls() {
  const bool fresh = m_daq && m_daq->running() && m_last_fresh_wall >= 0 &&
                     now() - m_last_fresh_wall <= freshness_s;
  const bool configured = m_joint->currentIndex() >= 0 && m_rpm->value() != 0 &&
                          !m_host->text().trimmed().isEmpty() &&
                          m_period->value() <= m_duration->value() &&
                          (m_kp->value() > 0 || m_kd->value() > 0);
  m_settings->setEnabled(!isBusy());
  m_start->setEnabled(m_available && !isBusy() && fresh && configured &&
                      m_frame.state == ControllerState::Connected);
  m_rearm->setEnabled(m_running && !m_supply && fresh &&
                      m_phase == Phase::Complete);
  m_stop->setEnabled(isBusy());
  m_export->setEnabled(m_valid && !m_supply && !m_dataset_pending &&
                       (!m_running || m_phase == Phase::Complete));
  if (!m_daq || !m_daq->running())
    m_live->setText(tr("External DAQ unavailable: Start disabled."));
}

void NoLoadTestDialog::start() {
  updateControls();
  if (!m_start->isEnabled())
    return;
  if (m_continuous->isChecked() &&
      QMessageBox::warning(
          this, tr("Confirm unlimited travel"),
          tr("Travel limits will be bypassed for this joint. Confirm that the "
             "shaft can rotate continuously, guards are in place, and the "
             "external Yaskawa is controlled safely."),
          QMessageBox::Yes | QMessageBox::Cancel,
          QMessageBox::Cancel) != QMessageBox::Yes)
    return;
  // Recheck after the confirmation's nested event loop.
  updateControls();
  if (!m_start->isEnabled())
    return;
  m_active_joint = static_cast<std::size_t>(m_joint->currentIndex());
  m_running = true;
  m_motion_ack = false;
  m_start_wall = now();
  m_phase = Phase::WaitingMotion;
  m_dwell_start = -1;
  m_status->setText(tr("Waiting for worker speed-hold acknowledgement..."));
  updateControls();
  emit busyChanged();
  emit speedHoldRequested(m_active_joint, m_rpm->value() * 6.0, m_ramp->value(),
                          m_continuous->isChecked(), false, m_kp->value(),
                          m_kd->value());
}

void NoLoadTestDialog::rearm() {
  if (!m_rearm->isEnabled())
    return;
  m_phase = Phase::Dwell;
  m_dwell_start = -1;
  m_status->setText(tr("Armed: waiting for continuous no-load dwell."));
  updateControls();
}

void NoLoadTestDialog::plotTelemetry() {
  const int index =
      m_running ? static_cast<int>(m_active_joint) : m_joint->currentIndex();
  if (index < 0 || static_cast<std::size_t>(index) >= m_frame.joints.size())
    return;
  const auto &joint = m_frame.joints[static_cast<std::size_t>(index)];
  const double t = now();
  m_speed_chart->append(m_series_ref_speed, t,
                        joint.speed_hold_active ? joint.ref_velocity_deg_s / 6.0
                                                : 0.0);
  m_speed_chart->append(m_series_drive_speed, t, joint.velocity_deg_s / 6.0);
}

void NoLoadTestDialog::plotPower() {
  m_power_chart->clearAll();
  if (m_supply_samples.empty()) {
    m_power_label->setText(tr("Power: no supply samples downloaded."));
    return;
  }
  double sum = 0.0;
  for (const auto &s : m_supply_samples) {
    m_power_chart->append(m_series_power, s.t_s, s.power_w);
    sum += s.power_w;
  }
  m_power_chart->setWindowSeconds(
      std::max(1.0, m_power_chart->dataMaxX() - m_power_chart->dataMinX()));
  m_power_label->setText(
      tr("Supply power (%1): mean %2 W over %3 samples, %4 s ELOG time.")
          .arg(m_valid ? tr("VALID") : tr("INVALID"))
          .arg(sum / static_cast<double>(m_supply_samples.size()), 0, 'f', 2)
          .arg(m_supply_samples.size())
          .arg(m_power_chart->dataMaxX() - m_power_chart->dataMinX(), 0, 'f',
               2));
}

void NoLoadTestDialog::appendTelemetry(const TelemetryFrame &frame) {
  const bool new_frame = frame.t_s != m_frame.t_s;
  if (new_frame || m_last_frame_wall < 0)
    m_last_frame_wall = now();
  m_frame = frame;
  if (new_frame)
    plotTelemetry();
  if (m_running) {
    if (frame.state == ControllerState::Faulted ||
        frame.state == ControllerState::Disconnected ||
        m_active_joint >= frame.joints.size()) {
      stopTest(tr("Controller fault, disconnect or missing joint"));
    } else {
      const auto &joint = frame.joints[m_active_joint];
      if (joint.fault || joint.hard_limit_violation ||
          (!m_continuous->isChecked() && joint.limit_violation)) {
        stopTest(tr("Joint fault or travel limit"));
      } else if (joint.speed_hold_active) {
        if (frame.state != ControllerState::Jogging ||
            std::fabs(joint.speed_hold_target_deg_s - m_rpm->value() * 6) >
                1e-6 ||
            joint.continuous_rotation != m_continuous->isChecked() ||
            joint.speed_hold_kp != m_kp->value() ||
            joint.speed_hold_kd != m_kd->value()) {
          stopTest(tr("Unexpected speed-hold command"));
        } else {
          m_motion_ack = true;
          if (m_phase == Phase::WaitingMotion)
            m_phase = Phase::Dwell;
        }
      } else if (m_motion_ack) {
        stopTest(tr("Worker stopped speed hold"));
      }
    }
  }
  updateControls();
}

void NoLoadTestDialog::buildMetadata() {
  m_metadata.clear();
  QTextStream metadata(&m_metadata);
  metadata.setRealNumberPrecision(17);
  metadata << "# session=" << m_session << "\n# joint_index=" << m_active_joint
           << "\n# target_rpm=" << m_rpm->value()
           << "\n# pvt_kp=" << m_kp->value() << "\n# pvt_kd=" << m_kd->value()
           << "\n# continuous_rotation=" << (m_continuous->isChecked() ? 1 : 0)
           << "\n# torque_band_nm=" << m_band->value()
           << "\n# speed_tolerance_rpm=" << m_tolerance->value()
           << "\n# dwell_s=" << m_dwell->value()
           << "\n# acquisition_duration_s=" << m_duration->value()
           << "\n# psu_period_s=" << m_period->value()
           << "\n# psu_host=" << m_host->text().trimmed()
           << "\n# sensor_interval=observed ELOG acquiring until reader "
              "finished; includes download latency\n";
  metadata.flush();
}

void NoLoadTestDialog::beginAcquisition() {
  m_sensor_samples.clear();
  m_supply_samples.clear();
  m_valid = false;
  m_cancelled = false;
  m_sensor_done = m_supply_done = m_supply_ok = m_interrupted = false;
  m_dataset_pending = true;
  m_capture_start = -1;
  m_session = QDateTime::currentDateTimeUtc().toString("yyyyMMdd-HHmmss-zzz") +
              "-" + QUuid::createUuid().toString(QUuid::Id128);
  buildMetadata();
  m_saved->clear();
  m_power_chart->clearAll();
  m_power_label->setText(tr("Power: acquiring..."));
  m_sensor_path.clear();
  m_supply_path.clear();
  PowerSupplyConfig config;
  config.host = m_host->text().trimmed().toStdString();
  config.sample_period_s = m_period->value();
  config.duration_s = m_duration->value();
  m_supply = std::make_unique<PowerSupplyReader>(config);
  m_phase = Phase::SupplyStarting;
  m_supply->start();
  m_status->setText(tr("No-load qualified. Starting finite PSU ELOG..."));
  emit busyChanged();
}

void NoLoadTestDialog::tick() {
  const double wall = now();
  if (m_last_fresh_wall < 0 || wall - m_last_fresh_wall > freshness_s)
    resetTorqueMean();
  if (m_running && m_last_fresh_wall >= 0 &&
      wall - m_last_fresh_wall > freshness_s)
    stopTest(tr("DAQ polling gap exceeded freshness limit"));
  bool distinct = false;
  ExternalDaqReader::Sample sample;
  if (m_daq && m_daq->running()) {
    sample = m_daq->latest();
    if (validSample(sample) && sample.t_s > m_last_daq_t) {
      distinct = true;
      m_last_daq_t = sample.t_s;
      m_last_fresh_wall = wall;
      updateTorqueReadout(sample, wall);
      m_speed_chart->append(m_series_daq_speed, wall, sample.speed_rpm);
      m_torque_chart->append(m_series_torque, wall, sample.torque_nm);
      m_torque_chart->append(m_series_band_hi, wall, m_band->value());
      m_torque_chart->append(m_series_band_lo, wall, -m_band->value());
    } else if (!validSample(sample) || sample.t_s < m_last_daq_t) {
      m_last_fresh_wall = -1;
      resetTorqueMean();
      m_live->setText(tr("Invalid external DAQ sample: mean unavailable."));
    }
  }
  if (m_daq && m_daq->running() && m_last_fresh_wall >= 0 &&
      wall - m_last_fresh_wall > freshness_s)
    m_live->setText(tr("Stale external DAQ samples: mean unavailable."));
  const bool fresh = m_daq && m_daq->running() && m_last_fresh_wall >= 0 &&
                     wall - m_last_fresh_wall <= freshness_s;
  if (m_running && (!fresh || wall - m_last_frame_wall > freshness_s))
    stopTest(tr("Stale or invalid DAQ / controller telemetry"));
  if (m_running && !m_motion_ack && wall - m_start_wall > m_ramp->value() + 2)
    stopTest(tr("Speed-hold command was not acknowledged"));

  if (m_supply) {
    const QString status = QString::fromStdString(m_supply->status());
    if (!m_cancelled)
      m_status->setText(status);
    if (m_phase == Phase::SupplyStarting &&
        status == QStringLiteral("ELOG: acquiring internally...")) {
      m_phase = Phase::Acquiring;
      m_capture_start = wall;
    }
    if (m_supply->finished()) {
      m_interrupted = m_supply->interrupted();
      m_supply_samples = m_supply->takeSamples();
      m_supply_ok =
          !m_interrupted && !m_supply_samples.empty() &&
          status.startsWith(QStringLiteral("ELOG complete:")) &&
          !status.contains(QStringLiteral("WARNING:")) &&
          !status.contains(QStringLiteral("error:"), Qt::CaseInsensitive);
      m_supply.reset();
      m_supply_done = true;
      if (!m_supply_ok && !m_cancelled)
        stopTest(
            tr("PSU failed or returned an incomplete buffer: %1").arg(status));
      if (m_capture_start < 0 && !m_cancelled)
        stopTest(tr("PSU acquisition start was not observed; dataset invalid"));
      emit busyChanged();
    }
  }
  if (m_running && distinct && m_motion_ack &&
      m_active_joint < m_frame.joints.size()) {
    const bool in_band =
        std::fabs(sample.torque_nm) <= m_band->value() &&
        std::fabs(sample.speed_rpm - m_rpm->value()) <= m_tolerance->value() &&
        std::isfinite(m_frame.joints[m_active_joint].velocity_deg_s) &&
        std::fabs(m_frame.joints[m_active_joint].velocity_deg_s / 6 -
                  m_rpm->value()) <= m_tolerance->value();
    if (m_phase == Phase::Dwell) {
      if (!in_band)
        m_dwell_start = -1;
      else if (m_dwell_start < 0)
        m_dwell_start = wall;
      else if (wall - m_dwell_start >= m_dwell->value()) {
        m_phase = Phase::SupplyStarting;
        beginAcquisition();
      }
      if (m_phase == Phase::Dwell)
        m_status->setText(
            tr("Armed: %1 s in band (required %2 s).")
                .arg(m_dwell_start < 0 ? 0 : wall - m_dwell_start, 0, 'f', 2)
                .arg(m_dwell->value()));
    } else if (m_phase == Phase::SupplyStarting ||
               (m_phase == Phase::Acquiring && !m_sensor_done)) {
      if (!in_band)
        stopTest(tr("No-load torque / speed left the valid band"));
      else if (m_phase == Phase::Acquiring && !m_sensor_done)
        m_sensor_samples.push_back(sample);
    }
  }
  if (m_supply_done)
    m_sensor_done = true;
  if (m_dataset_pending && m_supply_done && (m_sensor_done || m_cancelled))
    finishAcquisition();
  updateControls();
}

void NoLoadTestDialog::stopTest(const QString &reason) {
  if (m_dataset_pending) {
    m_cancelled = true;
    m_valid = false;
    m_sensor_done = true;
    m_result = reason;
  }
  if (m_supply)
    m_supply->requestStop();
  const bool release = m_running;
  m_running = false;
  m_phase = Phase::Idle;
  m_status->setText(
      reason + (m_supply
                    ? tr("\nCancelling ELOG / downloading partial buffer...")
                    : QString()));
  updateControls();
  if (release)
    emit speedHoldRequested(m_active_joint, 0, m_ramp->value(),
                            m_continuous->isChecked(), true, m_kp->value(),
                            m_kd->value());
  emit busyChanged();
}

void NoLoadTestDialog::finishAcquisition() {
  m_valid = !m_cancelled && m_supply_ok && !m_sensor_samples.empty();
  m_dataset_pending = false;
  m_result = m_valid
                 ? tr("Complete: no-load valid. DUT still rotating until Stop.")
                 : tr("INVALID acquisition: %1").arg(m_result);
  m_phase = m_running ? Phase::Complete : Phase::Idle;
  m_status->setText(m_result);
  plotPower();
  persistResults();
  updateControls();
}

bool NoLoadTestDialog::saveCsv(const QString &path, bool supply) {
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return false;
  QTextStream out(&file);
  out.setRealNumberPrecision(17);
  out << m_metadata << "# no_load_valid=" << (m_valid ? 1 : 0)
      << "\n# interrupted=" << (m_interrupted || m_cancelled ? 1 : 0)
      << "\n# result=" << m_result
      << "\n# synchronization=independent clocks; synchronize in "
         "post-processing\n";
  if (supply) {
    out << "# source=instrument ELOG samples\n"
           "elog_time_s,supply_voltage_v,supply_current_a\n";
    for (const auto &s : m_supply_samples)
      out << s.t_s << ',' << s.voltage_v << ',' << s.current_a << '\n';
  } else {
    out << "# source=polled latest DAQ samples; NOT full-rate acquisition\n"
           "daq_time_s,external_raw_torque_nm,external_filtered_torque_nm,"
           "external_speed_rpm\n";
    for (const auto &s : m_sensor_samples)
      out << s.t_s << ',' << s.torque_nm << ',' << s.filtered_torque_nm << ','
          << s.speed_rpm << '\n';
  }
  out.flush();
  return out.status() == QTextStream::Ok && file.commit();
}

void NoLoadTestDialog::persistResults() {
  const QString directory =
      QDir(m_result_root).filePath("no_load/" + m_session);
  if (m_result_root.isEmpty() || m_session.isEmpty() ||
      !QDir().mkpath(directory)) {
    m_saved->setText(tr("Automatic save failed in %1; data remains in memory.")
                         .arg(directory));
    return;
  }
  m_sensor_path =
      QDir(directory).filePath("no-load-" + m_session + "-sensor.csv");
  m_supply_path =
      QDir(directory).filePath("no-load-" + m_session + "-supply.csv");
  const bool sensor_ok = saveCsv(m_sensor_path, false);
  const bool supply_ok = saveCsv(m_supply_path, true);
  m_saved->setText(sensor_ok && supply_ok
                       ? tr("Saved %1 dataset:\n%2\n%3")
                             .arg(m_valid ? tr("valid") : tr("INVALID"),
                                  m_sensor_path, m_supply_path)
                       : tr("Automatic save failed; data remains in memory. "
                            "Check test directory: %1")
                             .arg(directory));
}

void NoLoadTestDialog::exportResults() {
  if (!m_export->isEnabled())
    return;
  for (bool supply : {false, true}) {
    const QString path = QFileDialog::getSaveFileName(
        this,
        supply ? tr("Export PSU CSV copy")
               : tr("Export polled sensor CSV copy"),
        supply ? m_supply_path : m_sensor_path, tr("CSV files (*.csv)"));
    if (!path.isEmpty() && !saveCsv(path, supply))
      QMessageBox::warning(this, tr("Export failed"),
                           tr("Could not save %1").arg(path));
  }
}

void NoLoadTestDialog::closeEvent(QCloseEvent *event) {
  stopTest(tr("Dialog closed"));
  QDialog::closeEvent(event);
}

void NoLoadTestDialog::reject() {
  stopTest(tr("Dialog dismissed"));
  QDialog::reject();
}
} // namespace actuator_test::gui
