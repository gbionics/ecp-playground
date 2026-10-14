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
#include <QSignalBlocker>
#include <QSpinBox>
#include <QTextStream>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <pwd.h>

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

QString NoLoadTestDialog::defaultResultRoot() {
  const QString env = qEnvironmentVariable("ACTUATOR_TEST_ANALYSIS_DIR");
  if (!env.isEmpty())
    return env;
  // Under sudo, keep saving into the invoking user's home rather than /root.
  QString home;
  if (const QByteArray sudo_user = qgetenv("SUDO_USER"); !sudo_user.isEmpty())
    if (const passwd *pw = getpwnam(sudo_user.constData()))
      home = QString::fromLocal8Bit(pw->pw_dir);
  if (home.isEmpty())
    home = QDir::homePath();
  return QDir(home).filePath(
      QStringLiteral("Documents/test-actuator-analysis"));
}

NoLoadTestDialog::NoLoadTestDialog(QWidget *parent, QString result_root)
    : QDialog(parent), m_result_root(std::move(result_root)) {
  setWindowTitle(tr("No-load speed test"));
  resize(1250, 760);
  m_clock.start();
  auto *layout = new QVBoxLayout(this);
  auto *instructions = new QLabel(
      tr("Acquisition does not require Start speed: the DUT may be controlled "
         "in current elsewhere while the load holds speed.\n"
         "Optional DUT PVT speed hold with temporary KP/KD. KP>0 tracks a "
         "position "
         "reference generated from speed; KP=0 uses velocity feedback only. "
         "Control the Yaskawa externally; this dialog "
         "does not command it.\nDAQ is mandatory. The "
         "symmetric no-load torque band and torsiometer speed tolerance are "
         "advisory "
         "only. Start acquisition manually, even outside the band.\n"
         "Completion does not change DUT speed. Set 0 rpm and Apply speed or "
         "use Soft stop to decelerate. Emergency stop releases the DUT "
         "immediately."));
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
  m_current_controls = new QGroupBox(tr("Manual DUT current"));
  auto *current_form = new QFormLayout(m_current_controls);
  m_current = new QDoubleSpinBox;
  m_current->setDecimals(6);
  m_current->setSuffix(tr(" A"));
  m_current->setKeyboardTracking(false);
  m_current->setToolTip(
      tr("Changes are applied immediately with the current ramp, including "
         "during acquisition. Signed current, limited to rated current; "
         "KP/KD are disabled by current control. This does not command the "
         "load."));
  auto *current_row = new QHBoxLayout;
  m_current_minus = new QPushButton(QStringLiteral("-"));
  m_current_plus = new QPushButton(QStringLiteral("+"));
  m_current_minus->setAccessibleName(tr("Decrease DUT current"));
  m_current_plus->setAccessibleName(tr("Increase DUT current"));
  current_row->addWidget(m_current_minus);
  current_row->addWidget(m_current, 1);
  current_row->addWidget(m_current_plus);
  current_form->addRow(tr("Target current"), current_row);
  m_current_step = new QDoubleSpinBox;
  m_current_step->setDecimals(6);
  m_current_step->setRange(0.001, 1);
  m_current_step->setValue(0.01);
  m_current_step->setSuffix(tr(" A"));
  current_form->addRow(tr("Adjustment step"), m_current_step);
  m_current->setSingleStep(m_current_step->value());
  m_current_ramp = new QDoubleSpinBox;
  m_current_ramp->setRange(0.01, 60);
  m_current_ramp->setValue(1);
  m_current_ramp->setSuffix(tr(" s"));
  current_form->addRow(tr("Current ramp"), m_current_ramp);
  left->addWidget(m_current_controls);
  m_current_readout = new QLabel;
  m_current_readout->setWordWrap(true);
  left->addWidget(m_current_readout);
  connect(m_current_step, &QDoubleSpinBox::valueChanged, this,
          [this](double value) {
            const double quantum = m_current->maximum() / 1000.0;
            if (quantum <= 0)
              return;
            const double step =
                std::max(1.0, std::round(value / quantum)) * quantum;
            const QSignalBlocker blocker(m_current_step);
            m_current_step->setValue(step);
            m_current->setSingleStep(step);
          });
  connect(m_current_minus, &QPushButton::clicked, m_current,
          &QDoubleSpinBox::stepDown);
  connect(m_current_plus, &QPushButton::clicked, m_current,
          &QDoubleSpinBox::stepUp);
  connect(m_current, &QDoubleSpinBox::valueChanged, this,
          [this] { applyCurrent(); });
  auto *live_speed = new QGroupBox(tr("Change speed while running"));
  auto *live_row = new QHBoxLayout(live_speed);
  m_live_rpm = new QDoubleSpinBox;
  m_live_rpm->setRange(-10000, 10000);
  m_live_rpm->setDecimals(3);
  m_live_rpm->setSuffix(tr(" rpm"));
  m_live_rpm->setToolTip(
      tr("New DUT speed, reached with the configured speed ramp. Changing "
         "speed restarts the dwell; during an acquisition it aborts it "
         "(partial data is saved) while the DUT keeps rotating."));
  m_apply_rpm = new QPushButton(tr("Apply speed"));
  live_row->addWidget(m_live_rpm, 1);
  live_row->addWidget(m_apply_rpm);
  left->addWidget(live_speed);
  m_live = new QLabel(tr("Waiting for fresh external DAQ samples."));
  m_band_status = new QLabel(tr("Torsiometer band: waiting for data."));
  m_status = new QLabel(tr("Idle"));
  m_saved = new QLabel;
  for (auto *label : {m_live, m_band_status, m_status, m_saved}) {
    label->setWordWrap(true);
    label->setTextInteractionFlags(Qt::TextSelectableByMouse);
    left->addWidget(label);
  }
  auto *buttons = new QGridLayout;
  m_start = new QPushButton(tr("Start speed"));
  m_acquire = new QPushButton(tr("Start acquisition"));
  m_acquire->setToolTip(
      tr("Start DC supply ELOG and sensor acquisition manually. The "
         "torsiometer band and dwell are advisory, not start conditions."));
  m_rearm = new QPushButton(tr("Rearm acquisition"));
  m_stop = new QPushButton(tr("Emergency stop"));
  m_stop->setStyleSheet(
      QStringLiteral("QPushButton { background-color: #b71c1c; color: white; "
                     "font-weight: bold; } "
                     "QPushButton:hover { background-color: #d32f2f; } "
                     "QPushButton:pressed { background-color: #8e1515; } "
                     "QPushButton:disabled { background-color: #633333; "
                     "color: #bbbbbb; }"));
  m_stop->setToolTip(
      tr("Immediately release the DUT and cancel any unfinished acquisition, "
         "without a deceleration ramp. Software stop only: does not command "
         "the external Yaskawa or guarantee a physical emergency stop."));
  m_soft_stop = new QPushButton(tr("Soft stop"));
  m_soft_stop->setToolTip(
      tr("Ramp to 0 rpm using Speed ramp and keep PVT control active. "
         "An unfinished acquisition is saved as partial data. This is not an "
         "emergency stop and does not control the Yaskawa."));
  m_export = new QPushButton(tr("Export CSV copies..."));
  buttons->addWidget(m_start, 0, 0);
  buttons->addWidget(m_soft_stop, 0, 1);
  buttons->addWidget(m_rearm, 1, 0);
  buttons->addWidget(m_export, 1, 1);
  buttons->addWidget(m_acquire, 2, 0, 1, 2);
  buttons->addWidget(m_stop, 3, 0, 1, 2);
  left->addLayout(buttons);
  left->addStretch(1);

  auto *body = new QHBoxLayout;
  body->addLayout(left, 2);
  body->addLayout(plots, 3);
  layout->addLayout(body, 1);
  connect(m_start, &QPushButton::clicked, this, &NoLoadTestDialog::start);
  connect(m_acquire, &QPushButton::clicked, this,
          &NoLoadTestDialog::startAcquisition);
  connect(m_stop, &QPushButton::clicked, this, [this] {
    const bool owns_motion = m_running;
    stopTest(tr("Emergency stop requested by user"));
    if (!owns_motion)
      emit emergencyStopRequested();
  });
  connect(m_soft_stop, &QPushButton::clicked, this,
          &NoLoadTestDialog::softStop);
  connect(m_rearm, &QPushButton::clicked, this, &NoLoadTestDialog::rearm);
  connect(m_apply_rpm, &QPushButton::clicked, this,
          &NoLoadTestDialog::applySpeed);
  connect(m_live_rpm, &QDoubleSpinBox::valueChanged, this,
          [this] { updateControls(); });
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
  const QSignalBlocker current_blocker(m_current);
  const double rated = valid && std::isfinite(m_joints[index].rated_current_a)
                           ? std::max(0.0, m_joints[index].rated_current_a)
                           : 0.0;
  m_current->setRange(-rated, rated);
  const QSignalBlocker step_blocker(m_current_step);
  const double quantum = rated / 1000.0;
  m_current_step->setRange(quantum, rated);
  m_current_step->setSingleStep(quantum);
  m_current_step->setValue(quantum);
  m_current->setSingleStep(quantum);
  m_current_step->setToolTip(
      tr("Minimum step: 1 permille of rated current (%1 A). "
         "Steps are rounded to integer multiples of this resolution.")
          .arg(quantum, 0, 'f', 6));
  m_current->setValue(
      valid && static_cast<std::size_t>(index) < m_frame.joints.size() &&
              quantum > 0
          ? std::round(m_frame.joints[index].commanded_current_a / quantum) *
                quantum
          : 0);
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

void NoLoadTestDialog::setDriveCapture(std::shared_ptr<DriveCapture> capture,
                                       double period_s) {
  if (isBusy())
    stopTest(tr("Drive capture source changed"));
  m_drive_capture = std::move(capture);
  m_drive_period_s = period_s;
  updateControls();
}

void NoLoadTestDialog::updateControls() {
  const double wall = now();
  const int index = m_joint->currentIndex();
  const bool telemetry_joint_present =
      index >= 0 && static_cast<std::size_t>(index) < m_frame.joints.size();
  const auto *joint =
      telemetry_joint_present ? &m_frame.joints[index] : nullptr;
  const auto *info =
      index >= 0 && static_cast<std::size_t>(index) < m_joints.size()
          ? &m_joints[index]
          : nullptr;
  const bool fresh = m_daq && m_daq->running() && m_last_fresh_wall >= 0 &&
                     wall - m_last_fresh_wall <= freshness_s;
  const bool telemetry_fresh = wall - m_last_frame_wall <= freshness_s;
  const bool controller_ready =
      m_frame.state != ControllerState::Disconnected &&
      m_frame.state != ControllerState::Faulted;
  const bool supply_configured = !m_host->text().trimmed().isEmpty() &&
                                 m_period->value() <= m_duration->value();
  const bool capture_idle = !m_dataset_pending && !m_supply;
  const bool drive_rate_valid =
      m_drive_capture && std::fabs(m_drive_period_s - 0.01) < 1e-9;
  const bool daq_rate_valid =
      m_daq && std::fabs(m_daq->publicationPeriodS() - 0.01) < 1e-9;
  const bool speed_acknowledged = m_motion_ack && !m_speed_change_pending;
  const bool speed_configured = index >= 0 && m_rpm->value() != 0 &&
                                (m_kp->value() > 0 || m_kd->value() > 0);
  if (!fresh)
    m_band_status->setText(tr("Torsiometer band: not monitoring."));
  m_settings->setEnabled(!isBusy());
  m_start->setEnabled(m_available && !isBusy() && fresh && supply_configured &&
                      speed_configured &&
                      m_frame.state == ControllerState::Connected);
  m_acquire->setEnabled(m_available && capture_idle && joint && !joint->fault &&
                        controller_ready && supply_configured && fresh &&
                        telemetry_fresh && (!m_running || speed_acknowledged) &&
                        drive_rate_valid && daq_rate_valid);
  if (capture_idle &&
      (!drive_rate_valid ||
       (m_daq && std::fabs(m_daq->publicationPeriodS() - 0.01) >= 1e-9)))
    m_status->setText(tr("Acquisition unavailable: DAQ and drive sources must "
                         "publish at 100 Hz."));
  m_rearm->setEnabled(m_running && !m_supply && fresh &&
                      !m_speed_change_pending && m_target_rpm != 0 &&
                      m_phase == Phase::Complete);
  const bool current_ready =
      joint && info && m_available && fresh && telemetry_fresh && !m_running &&
      m_frame.state != ControllerState::Faulted &&
      (m_frame.state == ControllerState::Connected ||
       (m_frame.state == ControllerState::Jogging &&
        joint->current_control_active)) &&
      info->selectable && info->driver_name == "MT_Device" &&
      info->rated_current_a > 0 &&
      joint->max_limit_deg > joint->min_limit_deg && !joint->fault &&
      std::none_of(m_frame.joints.begin(), m_frame.joints.end(),
                   [](const auto &j) { return j.speed_hold_active; });
  m_current_controls->setEnabled(current_ready);
  m_current_minus->setEnabled(m_current->value() > m_current->minimum());
  m_current_plus->setEnabled(m_current->value() < m_current->maximum());
  if (joint && info)
    m_current_readout->setText(
        tr("Drive commanded: %1 A | reported: %2 A | rated: %3 A%4")
            .arg(joint->commanded_current_a, 0, 'f', 6)
            .arg(joint->current_a, 0, 'f', 6)
            .arg(info->rated_current_a, 0, 'f', 3)
            .arg(current_ready ? QString()
                               : tr("\nCurrent adjustment unavailable: "
                                    "requires an idle/current "
                                    "MyActuator with travel limits, fresh data "
                                    "and no conflicting test.")));
  else
    m_current_readout->setText(tr("Current adjustment: no DUT selected."));
  m_stop->setEnabled(isBusy() ||
                     (m_frame.state != ControllerState::Disconnected));
  m_soft_stop->setEnabled(m_running && speed_acknowledged && m_target_rpm != 0);
  if (!m_running && current_ready && joint->current_control_active)
    m_soft_stop->setEnabled(true);
  m_live_rpm->setEnabled(m_running);
  m_apply_rpm->setEnabled(m_running && speed_acknowledged &&
                          m_live_rpm->value() != m_target_rpm);
  m_export->setEnabled(
      (!m_sensor_samples.empty() || !m_supply_samples.empty() ||
       !m_drive_samples.empty()) &&
      capture_idle && (!m_running || m_phase == Phase::Complete));
  if (!m_daq || !m_daq->running())
    m_live->setText(tr("External DAQ unavailable: Start disabled."));
}

void NoLoadTestDialog::applyCurrent() {
  updateControls();
  if (!m_current_controls->isEnabled())
    return;
  const double quantum = m_current->maximum() / 1000.0;
  const double current = std::round(m_current->value() / quantum) * quantum;
  const QSignalBlocker blocker(m_current);
  m_current->setValue(current);
  updateControls();
  emit currentRampRequested(static_cast<std::size_t>(m_joint->currentIndex()),
                            current, m_current_ramp->value());
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
  m_target_rpm = m_previous_rpm = m_rpm->value();
  m_speed_change_pending = false;
  m_live_rpm->setValue(m_target_rpm);
  m_running = true;
  m_motion_ack = false;
  m_start_wall = now();
  m_phase = Phase::WaitingMotion;
  m_dwell_start = -1;
  m_status->setText(tr("Waiting for worker speed-hold acknowledgement..."));
  updateControls();
  emit busyChanged();
  emit speedHoldRequested(m_active_joint, m_target_rpm * 6.0, m_ramp->value(),
                          m_continuous->isChecked(), false, m_kp->value(),
                          m_kd->value());
}

void NoLoadTestDialog::applySpeed() {
  updateControls();
  if (!m_apply_rpm->isEnabled())
    return;
  const double rpm = m_live_rpm->value();
  if (m_dataset_pending)
    abortAcquisition(tr("Speed changed during acquisition"));
  else if (m_phase == Phase::Dwell)
    m_dwell_start = -1;
  commandSpeed(rpm);
  if (m_phase != Phase::Complete)
    m_status->setText(
        tr("Speed changed to %1 rpm; dwell restarted.").arg(rpm, 0, 'f', 3));
  updateControls();
}

bool NoLoadTestDialog::inBand(const ExternalDaqReader::Sample &sample) const {
  return validSample(sample) && !m_speed_change_pending &&
         std::fabs(sample.torque_nm) <= m_band->value() &&
         std::fabs(sample.speed_rpm -
                   (m_running ? m_target_rpm : m_rpm->value())) <=
             m_tolerance->value();
}

void NoLoadTestDialog::startAcquisition() {
  // Consume the newest sensor sample and recheck freshness at the click.
  tick();
  if (!m_acquire->isEnabled())
    return;
  if (!m_running) {
    m_active_joint = static_cast<std::size_t>(m_joint->currentIndex());
    m_target_rpm = m_rpm->value();
  }
  m_sensor_samples.clear();
  m_drive_samples.clear();
  m_daq_dropped = m_drive_dropped = 0;
  m_drive_late_intervals = 0;
  m_drive_max_host_interval_s = 0;
  m_daq->captureBuffer().start();
  m_drive_capture->start(m_active_joint);
  beginAcquisition();
  updateControls();
}

void NoLoadTestDialog::commandSpeed(double rpm) {
  m_previous_rpm = m_target_rpm;
  m_target_rpm = rpm;
  m_speed_change_pending = true;
  m_speed_change_wall = now();
  emit speedHoldRequested(m_active_joint, m_target_rpm * 6.0, m_ramp->value(),
                          m_continuous->isChecked(), false, m_kp->value(),
                          m_kd->value());
  updateControls();
}

void NoLoadTestDialog::softStop() {
  updateControls();
  if (!m_soft_stop->isEnabled())
    return;
  if (!m_running) {
    if (m_dataset_pending)
      abortAcquisition(tr("Soft stop requested: ramping DUT current to 0 A."));
    const QSignalBlocker blocker(m_current);
    m_current->setValue(0);
    emit currentRampRequested(static_cast<std::size_t>(m_joint->currentIndex()),
                              0, m_current_ramp->value());
    m_status->setText(
        tr("Soft stop requested: ramping DUT current to 0 A. "
           "The load may continue rotating; coordinate its stop externally."));
    updateControls();
    return;
  }
  const QString reason = tr("Soft stop requested: commanding a ramp to 0 rpm; "
                            "PVT control remains active until Emergency stop.");
  if (m_dataset_pending)
    abortAcquisition(reason);
  else {
    m_phase = Phase::Complete;
    m_status->setText(reason);
  }
  m_dwell_start = -1;
  commandSpeed(0);
}

void NoLoadTestDialog::abortAcquisition(const QString &reason) {
  m_cancelled = true;
  m_acquisition_complete = false;
  m_result = reason;
  drainCapture(true);
  if (m_supply)
    m_supply->requestStop();
  else
    m_supply_done = true;
  // Keep rotating; Rearm is required for a new dataset once the download ends.
  m_phase = Phase::Complete;
  m_status->setText(reason +
                    tr("\nCancelling ELOG / downloading partial buffer..."));
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
  if (joint.speed_hold_active)
    m_speed_chart->append(m_series_ref_speed, t,
                          joint.ref_velocity_deg_s / 6.0);
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
      tr("Supply power: mean %1 W over %2 samples, %3 s ELOG time.")
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
  if (m_dataset_pending && !m_running &&
      (frame.state == ControllerState::Faulted ||
       frame.state == ControllerState::Disconnected ||
       m_active_joint >= frame.joints.size() ||
       frame.joints[m_active_joint].fault))
    stopTest(tr("Controller fault, disconnect or missing joint"));
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
        const auto commanded = [&](double rpm) {
          return std::fabs(joint.speed_hold_target_deg_s - rpm * 6) <= 1e-6;
        };
        if (commanded(m_target_rpm))
          m_speed_change_pending = false;
        if (frame.state != ControllerState::Jogging ||
            !(commanded(m_target_rpm) ||
              (m_speed_change_pending && commanded(m_previous_rpm))) ||
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
           << "\n# target_rpm=" << m_target_rpm << "\n# dut_motion_control="
           << (m_running ? "dialog PVT speed hold" : "external to this dialog")
           << "\n# pvt_kp=" << m_kp->value() << "\n# pvt_kd=" << m_kd->value()
           << "\n# dialog_gains_applied=" << (m_running ? 1 : 0)
           << "\n# continuous_rotation=" << (m_continuous->isChecked() ? 1 : 0)
           << "\n# torque_band_nm=" << m_band->value()
           << "\n# speed_tolerance_rpm=" << m_tolerance->value()
           << "\n# band_check=advisory; torsiometer torque and speed only"
           << "\n# dwell_s=" << m_dwell->value()
           << "\n# acquisition_duration_s=" << m_duration->value()
           << "\n# psu_period_s=" << m_period->value()
           << "\n# psu_host=" << m_host->text().trimmed()
           << "\n# torsiometer_tare_offset_nm="
           << (m_daq ? m_daq->tareOffsetNm() : 0.0)
           << "\n# daq_period_s=" << m_daq->publicationPeriodS()
           << "\n# daq_analog_sample_rate_hz=" << m_daq->analogSampleRateHz()
           << "\n# drive_period_s=" << m_drive_period_s
           << "\n# sensor_interval=manual acquisition request until reader "
              "finished or cancellation; includes PSU start/download latency\n";
  metadata.flush();
}

void NoLoadTestDialog::beginAcquisition() {
  m_supply_samples.clear();
  m_acquisition_complete = false;
  m_result.clear();
  m_cancelled = false;
  m_supply_done = m_supply_ok = m_interrupted = false;
  m_dataset_pending = true;
  m_session = QDateTime::currentDateTimeUtc().toString("yyyyMMdd-HHmmss-zzz") +
              "-" + QUuid::createUuid().toString(QUuid::Id128);
  buildMetadata();
  m_saved->clear();
  m_power_chart->clearAll();
  m_power_label->setText(tr("Power: acquiring..."));
  m_sensor_path.clear();
  m_supply_path.clear();
  m_drive_path.clear();
  PowerSupplyConfig config;
  config.host = m_host->text().trimmed().toStdString();
  config.sample_period_s = m_period->value();
  config.duration_s = m_duration->value();
  m_supply = std::make_unique<PowerSupplyReader>(config);
  m_phase = Phase::Capturing;
  m_supply->start();
  m_status->setText(tr("Starting finite PSU ELOG by user request..."));
  emit busyChanged();
}

void NoLoadTestDialog::tick() {
  const double wall = now();
  if (m_last_fresh_wall < 0 || wall - m_last_fresh_wall > freshness_s)
    resetTorqueMean();
  if ((m_running || m_dataset_pending) && m_last_fresh_wall >= 0 &&
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
  if ((m_running || m_dataset_pending) &&
      (!fresh || wall - m_last_frame_wall > freshness_s))
    stopTest(tr("Stale or invalid DAQ / controller telemetry"));
  if (m_running && !m_motion_ack && wall - m_start_wall > m_ramp->value() + 2)
    stopTest(tr("Speed-hold command was not acknowledged"));
  if (m_running && m_speed_change_pending &&
      wall - m_speed_change_wall > m_ramp->value() + 2)
    stopTest(tr("Speed change was not acknowledged"));

  if (m_supply) {
    const auto progress = m_supply->progress();
    const QString status = QString::fromStdString(progress.message);
    if (!m_cancelled)
      m_status->setText(status);
    if (m_supply->finished()) {
      const auto outcome = m_supply->progress();
      const QString result = QString::fromStdString(outcome.message);
      m_interrupted = m_supply->interrupted();
      m_supply_samples = m_supply->takeSamples();
      m_supply_ok = !m_interrupted && !m_supply_samples.empty() &&
                    outcome.state == PowerSupplyReader::State::Complete;
      m_supply.reset();
      m_supply_done = true;
      if (!m_supply_ok && !m_cancelled)
        stopTest(
            tr("PSU failed or returned an incomplete buffer: %1").arg(result));
      if (!outcome.acquisition_started && !m_cancelled)
        stopTest(tr("PSU acquisition start was not observed"));
      emit busyChanged();
    }
  }
  if (distinct && fresh) {
    const bool in_band = inBand(sample);
    if (!in_band)
      m_dwell_start = -1;
    else if (m_dwell_start < 0)
      m_dwell_start = wall;
    m_band_status->setText(
        (m_dwell_start >= 0 && wall - m_dwell_start >= m_dwell->value()
             ? tr("Torsiometer: in band for %1 s (dwell %2 s reached).")
             : (in_band ? tr("Torsiometer: in band for %1 s (dwell %2 s).")
                        : tr("Torsiometer: OUT OF BAND; %1 s in band "
                             "(dwell %2 s).")))
            .arg(m_dwell_start < 0 ? 0 : wall - m_dwell_start, 0, 'f', 2)
            .arg(m_dwell->value()));
  }
  if (m_dataset_pending && !m_cancelled && !m_supply_done && !drainCapture())
    stopTest(tr("Acquisition buffer overflow: DAQ dropped %1, drive dropped %2")
                 .arg(m_daq_dropped)
                 .arg(m_drive_dropped));
  if (m_dataset_pending && m_supply_done)
    finishAcquisition();
  updateControls();
}

bool NoLoadTestDialog::drainCapture(bool stop) {
  if (m_daq)
    m_daq_dropped = m_daq->captureBuffer().drain(m_sensor_samples, stop);
  if (m_drive_capture)
    m_drive_dropped = m_drive_capture->drain(m_drive_samples, stop);
  return m_daq_dropped == 0 && m_drive_dropped == 0;
}

void NoLoadTestDialog::stopTest(const QString &reason) {
  if (m_dataset_pending) {
    drainCapture(true);
    m_cancelled = true;
    m_acquisition_complete = false;
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
  if (!drainCapture(true)) {
    m_cancelled = true;
    m_result =
        tr("Acquisition buffer overflow: DAQ dropped %1, drive dropped %2")
            .arg(m_daq_dropped)
            .arg(m_drive_dropped);
  }
  m_acquisition_complete = !m_cancelled && m_supply_ok &&
                           !m_sensor_samples.empty() &&
                           !m_drive_samples.empty();
  m_drive_late_intervals = 0;
  m_drive_max_host_interval_s = 0;
  for (std::size_t i = 1; i < m_drive_samples.size(); ++i) {
    const double interval =
        m_drive_samples[i].host_time_s - m_drive_samples[i - 1].host_time_s;
    m_drive_max_host_interval_s =
        std::max(m_drive_max_host_interval_s, interval);
    if (interval > 1.5 * m_drive_period_s)
      ++m_drive_late_intervals;
  }
  m_dataset_pending = false;
  m_result =
      m_acquisition_complete
          ? (m_running
                 ? tr("Acquisition complete. DUT speed unchanged; "
                      "set 0 rpm and Apply speed or use Soft stop.")
                 : tr("Acquisition complete. DUT and load commands unchanged."))
          : tr("Acquisition interrupted or incomplete: %1")
                .arg(m_result.isEmpty() ? tr("Missing DAQ or drive samples")
                                        : m_result);
  if (m_drive_late_intervals)
    m_result += tr("\nTiming warning: %1 drive intervals exceeded %2 ms; "
                   "maximum host interval %3 ms.")
                    .arg(m_drive_late_intervals)
                    .arg(m_drive_period_s * 1500, 0, 'f', 1)
                    .arg(m_drive_max_host_interval_s * 1000, 0, 'f', 3);
  m_phase = m_running ? Phase::Complete : Phase::Idle;
  m_status->setText(m_result);
  plotPower();
  persistResults();
  updateControls();
  emit busyChanged();
}

bool NoLoadTestDialog::saveCsv(const QString &path, NoLoadCsvSource source,
                               QString &error) {
  const NoLoadDatasetView dataset{
      .metadata = m_metadata,
      .result = m_result,
      .interrupted = m_interrupted || m_cancelled,
      .joint = m_active_joint,
      .daq_dropped = m_daq_dropped,
      .drive_dropped = m_drive_dropped,
      .drive_period_s = m_drive_period_s,
      .drive_late_intervals = m_drive_late_intervals,
      .drive_max_host_interval_s = m_drive_max_host_interval_s,
      .sensor_samples = m_sensor_samples,
      .supply_samples = m_supply_samples,
      .drive_samples = m_drive_samples};
  return saveNoLoadCsv(path, source, dataset, error);
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
  m_drive_path =
      QDir(directory).filePath("no-load-" + m_session + "-drive.csv");
  QString sensor_error, supply_error, drive_error;
  const bool sensor_ok =
      saveCsv(m_sensor_path, NoLoadCsvSource::Sensor, sensor_error);
  const bool supply_ok =
      saveCsv(m_supply_path, NoLoadCsvSource::Supply, supply_error);
  const bool drive_ok =
      saveCsv(m_drive_path, NoLoadCsvSource::Drive, drive_error);
  m_saved->setText(
      sensor_ok && supply_ok && drive_ok
          ? tr("Saved dataset:\n%1\n%2\n%3")
                .arg(m_sensor_path, m_supply_path, m_drive_path)
          : tr("Automatic save failed; data remains in memory. "
               "Check test directory: %1\n%2\n%3\n%4")
                .arg(directory, sensor_error, supply_error, drive_error));
}

void NoLoadTestDialog::exportResults() {
  if (!m_export->isEnabled())
    return;
  for (const auto source : {NoLoadCsvSource::Sensor, NoLoadCsvSource::Supply,
                            NoLoadCsvSource::Drive}) {
    const bool supply = source == NoLoadCsvSource::Supply;
    const bool drive = source == NoLoadCsvSource::Drive;
    const QString path =
        QFileDialog::getSaveFileName(this,
                                     supply  ? tr("Export PSU CSV copy")
                                     : drive ? tr("Export drive CSV copy")
                                             : tr("Export DAQ CSV copy"),
                                     supply  ? m_supply_path
                                     : drive ? m_drive_path
                                             : m_sensor_path,
                                     tr("CSV files (*.csv)"));
    QString error;
    if (!path.isEmpty() && !saveCsv(path, source, error))
      QMessageBox::warning(this, tr("Export failed"),
                           tr("Could not save %1: %2").arg(path, error));
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
