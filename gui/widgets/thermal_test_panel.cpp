// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/thermal_test_panel.hpp"
#include "widgets/plot_panel.hpp"

#include <QAbstractItemView>
#include <QCheckBox>
#include <QCloseEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QPushButton>
#include <QSaveFile>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTextStream>
#include <QTimer>
#include <QUuid>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <stdexcept>

namespace actuator_test::gui {

namespace {

constexpr double kZeroCurrentEpsA = 1e-3;
// If telemetry never reports the 0 A command (e.g. the worker rejected it),
// stop recording anyway after this long.
constexpr double kZeroCurrentTimeoutS = 2.0;

QString tempText(int16_t t) {
  return t < 0 ? QStringLiteral("--") : QString::number(t);
}

QString optText(const std::optional<double> &v, int decimals) {
  return v ? QString::number(*v, 'f', decimals) : QStringLiteral("--");
}

/// Incremental least-squares line fit, x centred on `x0` for precision.
struct LineFit {
  double x0 = 0.0;
  double n = 0.0, sx = 0.0, sy = 0.0, sxx = 0.0, sxy = 0.0;
  double x_min = 0.0, x_max = 0.0;

  void add(double x, double y) {
    if (n == 0.0) {
      x_min = x_max = x;
    } else {
      x_min = std::min(x_min, x);
      x_max = std::max(x_max, x);
    }
    x -= x0;
    n += 1.0;
    sx += x;
    sy += y;
    sxx += x * x;
    sxy += x * y;
  }
  std::optional<double> mean() const {
    return n > 0.0 ? std::optional<double>(sy / n) : std::nullopt;
  }
  /// Slope in y-units per minute; needs a few seconds of spread.
  std::optional<double> slopePerMinute() const {
    if (n < 2.0 || x_max - x_min < 5.0)
      return std::nullopt;
    const double den = n * sxx - sx * sx;
    if (std::fabs(den) < 1e-12)
      return std::nullopt;
    return 60.0 * (n * sxy - sx * sy) / den;
  }
};

} // namespace

ThermalTestDialog::ThermalTestDialog(QWidget *parent,
                                   const QString &save_directory)
    : QDialog(parent), m_save_directory(save_directory) {
  setWindowTitle(tr("Copper Loss / Thermal Test"));
  setWindowFlags(Qt::Window | Qt::WindowCloseButtonHint |
                 Qt::WindowMinimizeButtonHint | Qt::WindowMaximizeButtonHint);
  resize(1400, 900);

  auto *root_layout = new QVBoxLayout(this);
  auto *splitter = new QSplitter(Qt::Horizontal, this);
  root_layout->addWidget(splitter);

  auto *left_scroll = new QScrollArea();
  left_scroll->setWidgetResizable(true);
  left_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  left_scroll->setMinimumWidth(400);
  left_scroll->setMaximumWidth(500);
  auto *left_panel = new QWidget();
  auto *layout = new QVBoxLayout(left_panel);
  left_scroll->setWidget(left_panel);
  splitter->addWidget(left_scroll);

  auto *intro = new QLabel(
      tr("<b>Copper loss / thermal model.</b> Apply constant current or a cyclic "
         "torque profile (no "
         "position/velocity feedback, same as the main window's current "
         "command). Temperatures are recorded from the moment current is "
         "injected through cooling at 0 A, until Release is pressed; the "
         "DC supply is polled and plotted on the same time axis."));
  intro->setWordWrap(true);
  layout->addWidget(intro);

  // --- Current command ----------------------------------------------------
  auto *cmd_box = new QGroupBox(tr("Current command"));
  auto *form = new QFormLayout(cmd_box);
  m_joint_combo = new QComboBox();
  form->addRow(tr("Joint:"), m_joint_combo);
  m_rated_label = new QLabel(tr("Rated: -- A"));
  form->addRow(QString(), m_rated_label);

  m_current_spin = new QDoubleSpinBox();
  m_current_spin->setRange(-100.0, 100.0);
  m_current_spin->setDecimals(2);
  m_current_spin->setSingleStep(0.1);
  m_current_spin->setSuffix(tr(" A"));
  m_current_spin->setAccessibleName(tr("Target current in amperes"));
  form->addRow(tr("Current:"), m_current_spin);

  m_ramp_spin = new QDoubleSpinBox();
  m_ramp_spin->setRange(0.0, 600.0);
  m_ramp_spin->setDecimals(2);
  m_ramp_spin->setSingleStep(0.5);
  m_ramp_spin->setValue(1.0);
  m_ramp_spin->setSuffix(tr(" s"));
  m_ramp_spin->setSpecialValueText(tr("Step (no ramp)"));
  m_ramp_spin->setAccessibleName(tr("Current ramp time in seconds"));
  m_ramp_spin->setToolTip(
      tr("Minimum-jerk ramp from the present setpoint to the new one, used "
         "by Apply and by Return to 0 A. 0 = step."));
  form->addRow(tr("Ramp time:"), m_ramp_spin);

  m_rate_spin = new QDoubleSpinBox();
  m_rate_spin->setRange(1.0, 60.0);
  m_rate_spin->setDecimals(0);
  m_rate_spin->setValue(5.0);
  m_rate_spin->setSuffix(tr(" Hz"));
  m_rate_spin->setToolTip(
      tr("Temperature sampling rate. Limited by the ~60 Hz GUI telemetry "
         "poll."));
  form->addRow(tr("Temperature sampling:"), m_rate_spin);

  m_confirm_check =
      new QCheckBox(tr("I confirm the rotor is mechanically locked / blocked"));
  form->addRow(m_confirm_check);

  auto *btn_row = new QHBoxLayout();
  m_apply_btn = new QPushButton(tr("A&pply current"));
  m_apply_btn->setToolTip(
      tr("Command the current above. The first non-zero command starts a new "
         "recording; later commands change the setpoint without restarting "
         "it (the averaging window restarts at each change)."));
  m_zero_btn = new QPushButton(tr("Return to 0 A"));
  m_zero_btn->setToolTip(
      tr("Command 0 A (still in current control) and keep recording cooling. "
         "Press Release to finish and save CSV files."));
  m_release_btn = new QPushButton(tr("Release"));
  m_release_btn->setToolTip(
      tr("Leave current control, idle the joint, and finish and save the "
         "recording."));
  btn_row->addWidget(m_apply_btn);
  btn_row->addWidget(m_zero_btn);
  btn_row->addWidget(m_release_btn);
  form->addRow(btn_row);
  layout->addWidget(cmd_box);

  auto *profile_box = new QGroupBox(tr("Cyclic torque - locked rotor"));
  auto *profile_form = new QFormLayout(profile_box);
  const auto profileSpin = [profile_form](const QString &label, double lo,
                                         double hi, double value, int decimals,
                                         const QString &suffix) {
    auto *spin = new QDoubleSpinBox();
    spin->setRange(lo, hi);
    spin->setDecimals(decimals);
    spin->setValue(value);
    spin->setSuffix(suffix);
    spin->setAccessibleName(label);
    profile_form->addRow(label, spin);
    return spin;
  };
  m_kt_spin = profileSpin(tr("Output Kt:"), 0.0, 10000.0, 0.0, 6, tr(" Nm/A"));
  m_kt_spin->setToolTip(
      tr("Output torque per commanded ampere, initially from the joint profile. "
         "Verify calibration and peak/RMS convention. Torque is estimated, "
         "not measured."));
  m_stance_spin = profileSpin(tr("Stance torque:"), -10000, 10000, 0, 2, tr(" Nm"));
  m_swing_spin = profileSpin(tr("Swing torque:"), -10000, 10000, 0, 2, tr(" Nm"));
  m_period_spin = profileSpin(tr("Cycle period:"), 0.2, 60, 1, 3, tr(" s"));
  m_duty_spin = profileSpin(tr("Stance fraction:"), 5, 95, 60, 1, tr(" %"));
  m_transition_spin =
      profileSpin(tr("Each transition:"), 0.01, 10, 0.1, 3, tr(" s"));
  m_duration_spin =
      profileSpin(tr("Cycling duration:"), 0.1, 86400, 600, 1, tr(" s"));
  m_profile_label = new QLabel();
  m_profile_label->setWordWrap(true);
  profile_form->addRow(m_profile_label);
  m_profile_preview = new StripChart(tr("Torque preview (Nm): two cycles"));
  m_profile_preview->setMinimumHeight(140);
  profile_form->addRow(m_profile_preview);
  m_profile_btn = new QPushButton(tr("Start cyclic profile"));
  profile_form->addRow(m_profile_btn);
  layout->addWidget(profile_box);

  m_status_label = new QLabel(tr("Idle."));
  m_status_label->setWordWrap(true);
  layout->addWidget(m_status_label);
  m_live_label = new QLabel();
  m_live_label->setWordWrap(true);
  m_live_label->setTextFormat(Qt::RichText);
  layout->addWidget(m_live_label);

  // --- DC supply polling --------------------------------------------------
  auto *psu_box = new QGroupBox(tr("DC power supply (polled)"));
  auto *psu_form = new QFormLayout(psu_box);
  m_psu_host_edit = new QLineEdit(
      QString::fromStdString(actuator_test::PowerSupplyConfig{}.host));
  psu_form->addRow(tr("Supply IP:"), m_psu_host_edit);
  m_psu_rate_spin = new QDoubleSpinBox();
  m_psu_rate_spin->setRange(0.1, 10.0);
  m_psu_rate_spin->setDecimals(1);
  m_psu_rate_spin->setValue(5.0);
  m_psu_rate_spin->setSuffix(tr(" Hz"));
  m_psu_rate_spin->setToolTip(
      tr("Each sample fetches the meter's latest V, I and P readings "
         "(FETC, ~1 ms each). The meter itself updates about every 200 ms, "
         "so rates above ~5 Hz only repeat readings."));
  psu_form->addRow(tr("Polling rate:"), m_psu_rate_spin);
  m_psu_auto_check =
      new QCheckBox(tr("Poll automatically while recording (Apply \u2192 Release)"));
  m_psu_auto_check->setChecked(true);
  m_psu_auto_check->setToolTip(
      tr("Starts polling when Apply current starts a recording and stops it "
         "when the recording ends, unless polling was started manually."));
  psu_form->addRow(m_psu_auto_check);
  m_psu_connect_btn = new QPushButton(tr("Start supply polling"));
  m_psu_connect_btn->setCheckable(true);
  m_psu_connect_btn->setToolTip(
      tr("Manual polling, e.g. to watch the supply before injecting current. "
         "Samples are stored only while a recording is active."));
  psu_form->addRow(m_psu_connect_btn);
  m_psu_status_label = new QLabel(
      tr("Read-only: no remote mode, output settings are never touched."));
  m_psu_status_label->setWordWrap(true);
  psu_form->addRow(m_psu_status_label);
  m_psu_live_label = new QLabel(QStringLiteral("--"));
  m_psu_live_label->setTextFormat(Qt::RichText);
  psu_form->addRow(m_psu_live_label);
  layout->addWidget(psu_box);

  auto *temperature_box = new QGroupBox(tr("PT1000 / AT20 - NI DAQ"));
  auto *temperature_form = new QFormLayout(temperature_box);
  m_temperature_channel = new QLineEdit(QStringLiteral("Dev1/ai0"));
  m_temperature_channel->setToolTip(
      tr("Differential input: AI0 (+), AI8 (-) on devices with this pairing. "
         "Verify your NI device pinout. AT20: 0-10 V = 0-150 C."));
  temperature_form->addRow(tr("Differential channel:"), m_temperature_channel);
  m_temperature_connect = new QPushButton(tr("Read PT1000"));
  m_temperature_connect->setCheckable(true);
  m_temperature_connect->setEnabled(external_daq_supported());
  temperature_form->addRow(m_temperature_connect);
  m_temperature_live = new QLabel(
      external_daq_supported() ? tr("0-10 V = 0-150 C; click Read PT1000.")
                               : tr("NI-DAQmx support unavailable in this build."));
  m_temperature_live->setWordWrap(true);
  temperature_form->addRow(m_temperature_live);
  layout->addWidget(temperature_box);
  connect(m_temperature_connect, &QPushButton::toggled, this,
          &ThermalTestDialog::toggleTemperatureDaq);
  auto *temperature_timer = new QTimer(this);
  temperature_timer->setInterval(100);
  connect(temperature_timer, &QTimer::timeout, this,
          &ThermalTestDialog::pollTemperatureDaq);
  temperature_timer->start();

  m_psu_timer = new QTimer(this);
  m_psu_timer->setInterval(100);
  connect(m_psu_timer, &QTimer::timeout, this,
          &ThermalTestDialog::pollPowerSupply);

  // --- Steady state & averages -------------------------------------------
  auto *ss_box = new QGroupBox(tr("Steady state && averages"));
  auto *ss_form = new QFormLayout(ss_box);
  m_ss_signal_combo = new QComboBox();
  m_ss_signal_combo->addItems(
      {tr("Motor temperature"), tr("Drive temperature"), tr("Motor and drive")});
  ss_form->addRow(tr("Criterion on:"), m_ss_signal_combo);
  m_ss_threshold_spin = new QDoubleSpinBox();
  m_ss_threshold_spin->setRange(0.001, 60.0);
  m_ss_threshold_spin->setDecimals(3);
  m_ss_threshold_spin->setSingleStep(0.05);
  m_ss_threshold_spin->setValue(0.1);
  m_ss_threshold_spin->setSuffix(tr(" \u00b0C/min"));
  ss_form->addRow(tr("Max |slope|:"), m_ss_threshold_spin);
  m_ss_window_spin = new QDoubleSpinBox();
  m_ss_window_spin->setRange(0.5, 240.0);
  m_ss_window_spin->setDecimals(1);
  m_ss_window_spin->setValue(10.0);
  m_ss_window_spin->setSuffix(tr(" min"));
  m_ss_window_spin->setToolTip(
      tr("Trailing window used for the slope fit and for every average. It "
         "never reaches back before the last current setpoint change. Steady "
         "state = window fully covered and |fitted slope| below threshold."));
  ss_form->addRow(tr("Window:"), m_ss_window_spin);
  m_ss_state_label = new QLabel(tr("Not recording"));
  m_ss_state_label->setAlignment(Qt::AlignCenter);
  m_ss_state_label->setStyleSheet(QStringLiteral(
      "font-weight:700; padding:4px; border-radius:4px; background:#374151; "
      "color:#e5e7eb;"));
  ss_form->addRow(m_ss_state_label);
  m_ss_detail_label = new QLabel();
  m_ss_detail_label->setWordWrap(true);
  m_ss_detail_label->setTextFormat(Qt::RichText);
  ss_form->addRow(m_ss_detail_label);
  m_mark_btn = new QPushButton(tr("Record steady-state point"));
  m_mark_btn->setToolTip(
      tr("Store the current window averages as a row of the table below."));
  ss_form->addRow(m_mark_btn);
  layout->addWidget(ss_box);

  m_points_table = new QTableWidget(0, 14);
  m_points_table->setHorizontalHeaderLabels(
      {tr("#"), tr("t (s)"), tr("Icmd (A)"), tr("I (A)"), tr("Motor (C)"),
       tr("Drive (C)"), tr("dTm (C/min)"), tr("dTd (C/min)"), tr("Vdc (V)"),
       tr("Idc (A)"), tr("Pdc (W)"), tr("\u03c3P (W)"), tr("Window (s)"),
       tr("Steady")});
  m_points_table->horizontalHeader()->setStretchLastSection(true);
  m_points_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_points_table->setSelectionMode(QAbstractItemView::NoSelection);
  m_points_table->setMinimumHeight(140);
  m_points_table->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Expanding);
  for (int col = 0; col < m_points_table->columnCount(); ++col)
    m_points_table->setColumnWidth(col, col == 0 ? 30 : 64);
  layout->addWidget(m_points_table, 1);

  m_export_btn = new QPushButton(tr("Export CSV..."));
  m_export_btn->setToolTip(
      tr("Save another copy. CSV files are also saved automatically when "
         "recording ends."));
  layout->addWidget(m_export_btn);

  // --- Right: plots -------------------------------------------------------
  auto *right_panel = new QWidget();
  auto *right_layout = new QVBoxLayout(right_panel);
  splitter->addWidget(right_panel);
  splitter->setStretchFactor(0, 0);
  splitter->setStretchFactor(1, 1);

  m_current_chart = new StripChart(tr("Commanded current (A)"));
  m_current_chart->setObjectName(QStringLiteral("thermalCurrentChart"));
  m_current_chart->setAxisTitles(tr("s since current injection"), tr("A"));
  m_current_chart->setMinimumHeight(160);
  m_current_chart->setPannable(true);
  m_current_chart->setMaxPoints(10'000'000);
  m_series_current =
      m_current_chart->addSeries(tr("commanded"), QColor(80, 190, 245));
  right_layout->addWidget(m_current_chart, 2);

  m_torque_live = new QLabel(tr("Torsiometer: waiting for DAQ data."));
  right_layout->addWidget(m_torque_live);
  auto *torque_controls = new QFormLayout();
  m_torque_window = new QDoubleSpinBox();
  m_torque_window->setRange(0.1, 60.0);
  m_torque_window->setDecimals(1);
  m_torque_window->setSingleStep(0.1);
  m_torque_window->setValue(1.0);
  m_torque_window->setSuffix(tr(" s"));
  m_torque_window->setAccessibleName(tr("Raw torque averaging window"));
  m_torque_window->setToolTip(
      tr("Signed mean of all raw DAQ samples over this trailing window. "
         "Whole acquisition blocks are used; startup windows may be partial."));
  torque_controls->addRow(tr("Raw torque mean window:"), m_torque_window);
  right_layout->addLayout(torque_controls);
  m_torque_chart = new StripChart(tr("Measured torque - torsiometer (Nm)"));
  m_torque_chart->setObjectName(QStringLiteral("thermalTorqueChart"));
  m_torque_chart->setAxisTitles(tr("s since current injection"), tr("Nm"));
  m_torque_chart->setMinimumHeight(140);
  m_torque_chart->setPannable(true);
  m_torque_chart->setMaxPoints(10'000'000);
  m_series_torque_raw =
      m_torque_chart->addSeries(tr("raw"), QColor(120, 140, 170));
  m_series_torque_filtered =
      m_torque_chart->addSeries(tr("filtered"), QColor(240, 200, 80));
  m_series_torque_mean =
      m_torque_chart->addSeries(tr("raw mean"), QColor(90, 235, 155));
  connect(m_torque_window, &QDoubleSpinBox::valueChanged, this, [this](double) {
    m_last_torque_time_s = 0.0;
  });
  right_layout->addWidget(m_torque_chart, 2);

  m_temperature_chart = new StripChart(tr("Temperature (C)"));
  m_temperature_chart->setAxisTitles(tr("s since current injection"), tr("C"));
  m_temperature_chart->setMinimumHeight(200);
  m_temperature_chart->setPannable(true);
  m_temperature_chart->setMaxPoints(10'000'000);
  m_series_motor =
      m_temperature_chart->addSeries(tr("motor"), QColor(245, 130, 70));
  m_series_drive =
      m_temperature_chart->addSeries(tr("drive"), QColor(180, 120, 245));
  m_series_pt1000 =
      m_temperature_chart->addSeries(tr("PT1000"), QColor(80, 220, 150));
  right_layout->addWidget(m_temperature_chart, 3);

  m_power_chart = new StripChart(tr("DC supply power (W, polled)"));
  m_power_chart->setAxisTitles(tr("s since current injection"), tr("W"));
  m_power_chart->setMinimumHeight(180);
  m_power_chart->setPannable(true);
  m_power_chart->setMaxPoints(10'000'000);
  m_series_power =
      m_power_chart->addSeries(tr("power"), QColor(240, 90, 90));
  m_series_power_mean = m_power_chart->addSeries(tr("window mean"),
                                                 QColor(250, 220, 120));
  right_layout->addWidget(m_power_chart, 2);
  updateChartWindows();

  connect(m_apply_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::applyCurrent);
  connect(m_profile_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::startCyclicProfile);
  for (auto *spin : {m_kt_spin, m_stance_spin, m_swing_spin, m_period_spin,
                     m_duty_spin, m_transition_spin, m_duration_spin, m_ramp_spin})
    connect(spin, &QDoubleSpinBox::valueChanged, this,
            [this](double) { refreshProfilePreview(); });
  connect(m_zero_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::returnToZero);
  connect(m_release_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::release);
  connect(m_psu_connect_btn, &QPushButton::toggled, this,
          &ThermalTestDialog::togglePowerSupply);
  connect(m_mark_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::recordSteadyPoint);
  connect(m_export_btn, &QPushButton::clicked, this,
          &ThermalTestDialog::exportCsv);
  connect(m_confirm_check, &QCheckBox::toggled, this,
          [this](bool) { updateControls(); });
  connect(m_joint_combo, &QComboBox::currentIndexChanged, this, [this](int) {
    updateRatedCurrent();
    refreshProfilePreview();
    updateControls();
  });
  for (auto *spin : {m_ss_window_spin, m_ss_threshold_spin})
    connect(spin, &QDoubleSpinBox::valueChanged, this,
            [this](double) { refreshStats(); });
  connect(m_ss_signal_combo, &QComboBox::currentIndexChanged, this,
          [this](int) { refreshStats(); });

  refreshProfilePreview();
  updateControls();
}

void ThermalTestDialog::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;
  const QString previous = m_joint_combo->currentText();
  m_joint_combo->blockSignals(true);
  m_joint_combo->clear();
  for (const auto &j : joints)
    m_joint_combo->addItem(QString::fromStdString(j.name));
  const int idx = m_joint_combo->findText(previous);
  m_joint_combo->setCurrentIndex(idx >= 0 ? idx : 0);
  m_joint_combo->blockSignals(false);
  updateRatedCurrent();
  updateControls();
}

bool ThermalTestDialog::isBusy() const {
  return m_running || m_poller != nullptr || m_temperature_daq != nullptr ||
         m_shared_temperature_enabled;
}

void ThermalTestDialog::setExternalDaqReader(
    std::shared_ptr<ExternalDaqReader> reader) {
  m_shared_daq = std::move(reader);
}

std::optional<ExternalDaqReader::Sample> ThermalTestDialog::latestTorque() const {
  if (!m_shared_daq || !m_shared_daq->running())
    return std::nullopt;
  const auto sample = m_shared_daq->latest();
  const double age = PowerSupplyPoller::steadyNowS() - sample.steady_time_s;
  if (sample.sample_index == 0 || age < 0.0 || age > 1.0 ||
      !std::isfinite(sample.torque_nm) ||
      !std::isfinite(sample.filtered_torque_nm))
    return std::nullopt;
  return sample;
}

void ThermalTestDialog::toggleTemperatureDaq(bool enabled) {
  m_pt1000.reset();
  m_shared_temperature_enabled = false;
  if (m_temperature_daq) {
    m_temperature_daq->stop();
    const auto error = m_temperature_daq->lastError();
    m_temperature_daq.reset();
    if (!error.empty())
      showWarning(tr("PT1000 DAQ"), QString::fromStdString(error));
  }
  if (enabled) {
    const auto channel = m_temperature_channel->text().trimmed().toStdString();
    if (m_shared_daq && m_shared_daq->temperatureChannel() == channel) {
      if (!m_shared_daq->running()) {
        showWarning(tr("PT1000 DAQ"),
                    tr("Shared DAQ acquisition is not running: %1")
                        .arg(QString::fromStdString(m_shared_daq->lastError())));
        const QSignalBlocker blocker(m_temperature_connect);
        m_temperature_connect->setChecked(false);
        m_temperature_live->setText(tr("PT1000 connection failed."));
        return;
      }
      m_shared_temperature_enabled = true;
      m_temperature_channel->setEnabled(false);
      m_temperature_connect->setText(tr("Disconnect PT1000"));
      m_temperature_live->setText(
          tr("Reading PT1000 from the shared torsiometer DAQ task."));
      return;
    }
    if (m_shared_daq && m_shared_daq->running()) {
      showWarning(tr("PT1000 DAQ"),
                  tr("The analog DAQ is already in use. Its shared PT1000 "
                     "channel is %1; select that channel to avoid a second task.")
                      .arg(QString::fromStdString(m_shared_daq->temperatureChannel())));
      const QSignalBlocker blocker(m_temperature_connect);
      m_temperature_connect->setChecked(false);
      return;
    }
    auto reader = std::make_unique<TemperatureDaqReader>(
        channel);
    std::string error;
    if (!reader->start(error)) {
      showWarning(tr("PT1000 DAQ"), QString::fromStdString(error));
      const QSignalBlocker blocker(m_temperature_connect);
      m_temperature_connect->setChecked(false);
      m_temperature_live->setText(tr("PT1000 connection failed."));
      return;
    }
    m_temperature_daq = std::move(reader);
    m_temperature_live->setText(tr("Waiting for PT1000 sample..."));
  } else {
    m_temperature_live->setText(tr("PT1000 disconnected."));
  }
  m_temperature_channel->setEnabled(!m_temperature_daq);
  m_temperature_connect->setText(
      m_temperature_daq ? tr("Disconnect PT1000") : tr("Read PT1000"));
}

void ThermalTestDialog::pollTemperatureDaq() {
  if (!m_temperature_daq && !m_shared_temperature_enabled)
    return;
  if (m_shared_temperature_enabled) {
    if (!m_shared_daq->running()) {
      const auto error = m_shared_daq->lastError();
      m_temperature_connect->setChecked(false);
      showWarning(tr("PT1000 DAQ"), QString::fromStdString(error));
      return;
    }
    m_pt1000 = m_shared_daq->latest().temperature;
  } else if (!m_temperature_daq->running()) {
    m_temperature_connect->setChecked(false);
    return;
  } else {
    m_pt1000 = m_temperature_daq->latest();
  }
  if (!m_pt1000)
    return;
  const double age = PowerSupplyPoller::steadyNowS() - m_pt1000->steady_time_s;
  if (age > 1.0) {
    m_pt1000.reset();
    m_temperature_live->setText(tr("PT1000 sample stale; not recording it."));
    return;
  }
  m_temperature_live->setText(
      tr("%1 C (%2 V)%3")
          .arg(m_pt1000->temperature_c, 0, 'f', 2)
          .arg(m_pt1000->voltage_v, 0, 'f', 4)
          .arg(m_pt1000->voltage_v < 0.0 || m_pt1000->voltage_v > 10.0
                   ? tr(" - outside configured 0-10 V range") : QString()));
}

std::size_t ThermalTestDialog::selectedJoint() const {
  return static_cast<std::size_t>(std::max(0, m_joint_combo->currentIndex()));
}

double ThermalTestDialog::testTimeS() const {
  return m_t0_s ? m_latest_frame_t_s - *m_t0_s : 0.0;
}

double ThermalTestDialog::windowS() const {
  return m_ss_window_spin->value() * 60.0;
}

void ThermalTestDialog::updateRatedCurrent() {
  const std::size_t j = selectedJoint();
  if (j >= m_joints.size()) {
    m_rated_label->setText(tr("Rated: -- A"));
    m_kt_joint.clear();
    return;
  }
  const double rated = m_joints[j].rated_current_a;
  const QString name = QString::fromStdString(m_joints[j].name);
  if (!m_running && m_kt_joint != name) {
    m_kt_joint = name;
    m_kt_spin->setValue(std::isfinite(m_joints[j].torque_constant_nm_per_a)
                           ? std::max(0.0, m_joints[j].torque_constant_nm_per_a)
                           : 0.0);
  }
  m_rated_label->setText(rated > 0.0
                             ? tr("Rated: %1 A").arg(rated, 0, 'f', 2)
                             : tr("Rated: unavailable"));
}

void ThermalTestDialog::updateControls() {
  const bool have_joint =
      !m_joints.empty() && m_joint_combo->currentIndex() >= 0;
  m_joint_combo->setEnabled(!m_running);
  m_rate_spin->setEnabled(!m_running);
  m_apply_btn->setEnabled(have_joint && m_confirm_check->isChecked());
  m_zero_btn->setEnabled(have_joint);
  m_release_btn->setEnabled(have_joint);
  m_psu_host_edit->setEnabled(!m_poller);
  m_psu_rate_spin->setEnabled(!m_poller);
  m_mark_btn->setEnabled(m_running && !m_samples.empty());
  m_export_btn->setEnabled(!m_running && !m_samples.empty());
  m_profile_btn->setEnabled(have_joint && m_confirm_check->isChecked() &&
                            !m_profile_requested);
}

actuator_test::CyclicCurrentProfile ThermalTestDialog::configuredProfile() const {
  CyclicCurrentProfile p;
  const double kt = m_kt_spin->value();
  p.stance_a = kt > 0.0 ? m_stance_spin->value() / kt : 0.0;
  p.swing_a = kt > 0.0 ? m_swing_spin->value() / kt : 0.0;
  p.period_s = m_period_spin->value();
  p.stance_fraction = m_duty_spin->value() / 100.0;
  p.transition_s = m_transition_spin->value();
  p.entry_exit_s = m_ramp_spin->value();
  p.duration_s = m_duration_spin->value();
  return p;
}

void ThermalTestDialog::refreshProfilePreview() {
  const auto p = configuredProfile();
  m_profile_preview->resetSeries();
  const int series = m_profile_preview->addSeries(tr("torque"), QColor(245, 180, 70));
  if (m_kt_spin->value() <= 0.0 || !p.valid()) {
    m_profile_label->setText(
        tr("Set Kt > 0, Ramp time > 0, and transition time no longer than "
           "either cycle phase."));
    m_profile_preview->update();
    return;
  }
  m_profile_label->setText(
      tr("Stance %1 A; swing %2 A; cycle RMS %3 A (theoretical).\n"
         "Entry/exit ramp: %4 s each, then cooling at 0 A until Release. "
         "Settings are applied only on Start.")
          .arg(p.stance_a, 0, 'f', 3).arg(p.swing_a, 0, 'f', 3)
          .arg(p.cycleRmsA(), 0, 'f', 3).arg(p.entry_exit_s, 0, 'f', 2));
  m_profile_preview->setWindowSeconds(2.0 * p.period_s);
  for (int i = 0; i <= 400; ++i) {
    const double t = i * p.period_s / 200.0;
    m_profile_preview->append(series, t, p.cycleValue(t) * m_kt_spin->value());
  }
}

void ThermalTestDialog::finishCyclicProfile() {
  if (!m_profile_requested)
    return;
  m_profile_requested = false;
  if (!m_profiles.empty())
    m_profiles.back().ended_s = testTimeS();
  updateControls();
}

void ThermalTestDialog::startCyclicProfile() {
  if (m_profile_requested || selectedJoint() >= m_joints.size() ||
      !m_confirm_check->isChecked())
    return;
  const auto p = configuredProfile();
  const double rated = m_joints[selectedJoint()].rated_current_a;
  if (m_kt_spin->value() <= 0.0 || !p.valid() ||
      !std::isfinite(rated) || rated <= 0.0 ||
      std::max(std::fabs(p.stance_a), std::fabs(p.swing_a)) > rated ||
      (p.stance_a == 0.0 && p.swing_a == 0.0)) {
    showWarning(tr("Cyclic torque"),
                tr("Invalid profile. Verify Kt, non-zero torque levels, positive "
                   "ramp time and phase/transition durations. Both current levels "
                   "must be within the joint's rated current."));
    return;
  }
  if (!m_running)
    startRecording();
  m_returning_to_zero = false;
  m_zero_requested_t_s.reset();
  m_setpoint_t_s = testTimeS() + p.entry_exit_s;
  m_steady = false;
  m_steady_since_s.reset();
  m_profiles.push_back({testTimeS(), std::nullopt, std::nullopt,
                        m_kt_spin->value(), p});
  m_profile_requested = true;
  emit cyclicCurrentRequested(CyclicCurrentCommand{m_active_joint, p});
  m_status_label->setText(
      tr("Recording cyclic torque for %1 s, plus entry/exit ramps. "
         "Return to 0 A stops the profile and records cooling.")
          .arg(p.duration_s, 0, 'f', 1));
  updateControls();
}

void ThermalTestDialog::applyCurrent() {
  if (m_joints.empty())
    return;
  const double target = m_current_spin->value();
  if (std::fabs(target) < kZeroCurrentEpsA) {
    returnToZero();
    return;
  }
  const double ramp = m_ramp_spin->value();
  finishCyclicProfile();
  if (!m_running) {
    startRecording();
  } else {
    m_setpoint_t_s = testTimeS();
    m_steady_since_s.reset();
  }
  // A new setpoint restarts the steady-state / averaging window once the
  // ramp has reached it.
  m_setpoint_t_s += ramp;
  m_returning_to_zero = false;
  m_zero_requested_t_s.reset();
  if (ramp > 0.0)
    emit currentRampRequested(m_active_joint, target, ramp);
  else
    emit currentSetpointRequested(m_active_joint, target, /*release=*/false);
  m_status_label->setText(
      tr("Recording: commanding %1 A on %2%3.")
          .arg(target, 0, 'f', 2)
          .arg(m_joint_combo->itemText(static_cast<int>(m_active_joint)))
          .arg(ramp > 0.0 ? tr(" (ramp %1 s)").arg(ramp, 0, 'f', 2)
                          : QString()));
  refreshStats();
}

void ThermalTestDialog::returnToZero() {
  const std::size_t joint = m_running ? m_active_joint : selectedJoint();
  if (joint >= m_joints.size())
    return;
  finishCyclicProfile();
  const double ramp = m_ramp_spin->value();
  if (ramp > 0.0)
    emit currentRampRequested(joint, 0.0, ramp);
  else
    emit currentSetpointRequested(joint, 0.0, /*release=*/false);
  if (m_running) {
    m_setpoint_t_s = testTimeS() + ramp;
    m_steady = false;
    m_steady_since_s.reset();
    m_returning_to_zero = true;
    m_zero_requested_t_s.reset();
    m_zero_ramp_s = ramp;
    m_status_label->setText(
        ramp > 0.0 ? tr("Ramping to 0 A over %1 s...").arg(ramp, 0, 'f', 2)
                   : tr("Returning to 0 A..."));
  }
}

void ThermalTestDialog::release() {
  const std::size_t joint = m_running ? m_active_joint : selectedJoint();
  if (joint >= m_joints.size())
    return;
  emit currentSetpointRequested(joint, 0.0, /*release=*/true);
  if (m_running)
    finishRecording(tr("current control released"));
}

void ThermalTestDialog::startRecording() {
  if (m_psu_auto_check->isChecked() && !m_poller) {
    m_psu_connect_btn->setChecked(true);
    m_psu_auto_started = m_poller != nullptr;
  }
  m_active_joint = selectedJoint();
  m_running = true;
  m_returning_to_zero = false;
  m_control_observed = false;
  m_steady = false;
  m_t0_s.reset();
  m_t0_steady_s.reset();
  m_next_sample_t_s.reset();
  m_zero_requested_t_s.reset();
  m_steady_since_s.reset();
  m_setpoint_t_s = 0.0;
  m_samples.clear();
  m_current_chart_last_s.reset();
  m_last_torque_time_s = 0.0;
  m_samples.reserve(200'000);
  m_power.clear();
  m_points.clear();
  m_profiles.clear();
  m_points_table->setRowCount(0);
  for (auto *chart : {m_current_chart, m_torque_chart, m_temperature_chart,
                      m_power_chart}) {
    chart->clearAll();
    chart->followLatest();
  }
  updateChartWindows();
  refreshStats();
  updateControls();
}

void ThermalTestDialog::finishRecording(const QString &reason) {
  if (!m_running)
    return;
  // Drain while recording is still active so pending supply samples are saved.
  pollPowerSupply();
  finishCyclicProfile();
  m_running = false;
  m_returning_to_zero = false;
  m_zero_requested_t_s.reset();
  if (m_psu_auto_started) {
    m_psu_auto_started = false;
    m_psu_connect_btn->setChecked(false);
  }
  m_status_label->setText(
      tr("Recording stopped (%1). %2 temperature and %3 supply samples over "
         "%4 s.")
          .arg(reason)
          .arg(m_samples.size())
          .arg(m_power.size())
          .arg(m_samples.empty() ? 0.0 : m_samples.back().t_s, 0, 'f', 1));
  refreshStats();
  updateControls();
  autoSaveCsv();
}

void ThermalTestDialog::appendTelemetry(const TelemetryFrame &frame) {
  m_latest_frame_t_s = frame.t_s;
  const std::size_t joint = m_running ? m_active_joint : selectedJoint();
  if (joint >= frame.joints.size()) {
    if (m_running)
      finishRecording(tr("joint disappeared from telemetry"));
    return;
  }
  const JointTelemetry &jt = frame.joints[joint];
  updateLiveReadout(jt);
  const auto torque = latestTorque();
  const auto torque_mean = torque
      ? m_shared_daq->torqueMean(m_torque_window->value())
      : std::nullopt;
  if (torque) {
    QString text = tr("Torsiometer: %1 Nm raw; %2 Nm filtered")
                              .arg(torque->torque_nm, 0, 'f', 2)
                              .arg(torque->filtered_torque_nm, 0, 'f', 2);
    if (torque_mean)
      text += tr("\nRaw mean (%1 s): %2 Nm [%3 s acquired, %4 samples]")
                  .arg(m_torque_window->value(), 0, 'f', 1)
                  .arg(torque_mean->torque_nm, 0, 'f', 2)
                  .arg(torque_mean->span_s, 0, 'f', 2)
                  .arg(static_cast<qulonglong>(torque_mean->samples));
    m_torque_live->setText(text);
  } else {
    m_torque_live->setText(
        tr("Torsiometer: unavailable or stale DAQ data (not recorded)."));
  }
  if (!m_running)
    return;

  if (!m_t0_s) {
    m_t0_s = frame.t_s;
    m_t0_steady_s = actuator_test::PowerSupplyPoller::steadyNowS();
  }
  const double t = frame.t_s - *m_t0_s;
  if (torque && torque->steady_time_s > m_last_torque_time_s) {
    m_torque_chart->append(m_series_torque_raw, t, torque->torque_nm);
    m_torque_chart->append(m_series_torque_filtered, t,
                           torque->filtered_torque_nm);
    if (torque_mean)
      m_torque_chart->append(m_series_torque_mean, t, torque_mean->torque_nm);
    m_last_torque_time_s = torque->steady_time_s;
  }
  if (!m_current_chart_last_s || t > *m_current_chart_last_s) {
    m_current_chart->append(m_series_current, t, jt.commanded_current_a);
    m_current_chart_last_s = t;
    updateChartWindows();
  }

  if (jt.fault) {
    recordSample(t, jt);
    emit currentSetpointRequested(m_active_joint, 0.0, /*release=*/true);
    finishRecording(
        tr("joint '%1' faulted").arg(QString::fromStdString(jt.name)));
    return;
  }

  if (jt.current_control_active) {
    m_control_observed = true;
  } else if (m_control_observed) {
    recordSample(t, jt);
    finishRecording(tr("current control ended outside this view"));
    return;
  }

  if (m_profile_requested) {
    auto &run = m_profiles.back();
    if (jt.cyclic_current_active) {
      if (!run.started_s) {
        run.started_s = t;
        m_setpoint_t_s = t + run.profile.entry_exit_s;
      }
    } else if (run.started_s) {
      finishCyclicProfile();
      m_setpoint_t_s = t;
      m_steady = false;
      m_steady_since_s.reset();
      m_status_label->setText(
          tr("Cyclic profile ended. Recording continues; press Release to save."));
      refreshStats();
    } else if (t - run.requested_s > 2.0) {
      emit currentSetpointRequested(m_active_joint, 0.0, /*release=*/true);
      showWarning(tr("Cyclic torque"),
                  tr("The worker did not confirm the cyclic profile. Check the "
                     "controller log (limits, current rating, connection)."));
      recordSample(t, jt);
      finishRecording(tr("cyclic profile was not confirmed"));
      return;
    }
  }

  if (m_returning_to_zero) {
    if (!m_zero_requested_t_s)
      m_zero_requested_t_s = frame.t_s;
    if (!jt.current_transition_active && !jt.cyclic_current_active &&
        std::fabs(jt.commanded_current_a) < kZeroCurrentEpsA) {
      m_returning_to_zero = false;
      m_zero_requested_t_s.reset();
      m_setpoint_t_s = t;
      m_steady_since_s.reset();
      m_status_label->setText(
          tr("Recording cooling at 0 A. Press Release to finish and save CSV."));
      refreshStats();
    } else if (const double timeout = kZeroCurrentTimeoutS + m_zero_ramp_s;
               frame.t_s - *m_zero_requested_t_s > timeout) {
      recordSample(t, jt);
      finishRecording(tr("0 A requested; drive did not confirm within %1 s")
                          .arg(timeout, 0, 'f', 1));
      return;
    }
  }

  // Fixed-rate decimation of the GUI telemetry poll; the schedule advances by
  // whole periods so the average rate matches the request.
  const double period = 1.0 / m_rate_spin->value();
  if (m_next_sample_t_s && t < *m_next_sample_t_s)
    return;
  recordSample(t, jt);
  m_next_sample_t_s =
      m_next_sample_t_s ? *m_next_sample_t_s + period : t + period;
  if (*m_next_sample_t_s <= t)
    m_next_sample_t_s = t + period;
}

void ThermalTestDialog::recordSample(double t_s, const JointTelemetry &jt) {
  if (!m_samples.empty() && t_s <= m_samples.back().t_s)
    return;
  if (m_samples.empty()) {
    m_commanded_i2_origin = jt.commanded_i2_a2s;
    m_actual_i2_origin = jt.actual_i2_a2s;
  }
  pollTemperatureDaq();
  m_samples.push_back({t_s, jt.commanded_current_a, jt.current_a,
                       jt.motor_temp_c, jt.drive_temp_c, m_steady,
                       jt.cyclic_current_active,
                       jt.commanded_i2_a2s - m_commanded_i2_origin,
                       jt.actual_i2_a2s - m_actual_i2_origin, m_pt1000,
                       latestTorque()});
  auto &sample = m_samples.back();
  sample.torque_mean_window_s = m_torque_window->value();
  if (sample.torsiometer)
    sample.torque_mean = m_shared_daq->torqueMean(sample.torque_mean_window_s);
  if (m_pt1000)
    m_temperature_chart->append(m_series_pt1000, t_s, m_pt1000->temperature_c);
  if (jt.motor_temp_c >= 0)
    m_temperature_chart->append(m_series_motor, t_s, jt.motor_temp_c);
  if (jt.drive_temp_c >= 0)
    m_temperature_chart->append(m_series_drive, t_s, jt.drive_temp_c);
  updateChartWindows();
  // Refresh about twice a second whatever the sampling rate; the window scan
  // is cheap but needless on every sample.
  const auto every = static_cast<std::size_t>(
      std::max(1.0, std::round(m_rate_spin->value() / 2.0)));
  if (m_samples.size() % every == 1 % every)
    refreshStats();
  if (m_samples.size() == 1)
    updateControls();
}

void ThermalTestDialog::updateLiveReadout(const JointTelemetry &jt) {
  m_live_label->setText(
      tr("<b>t</b> = %1 s &nbsp; <b>I<sub>cmd</sub></b> = %2 A &nbsp; "
         "<b>I</b> = %3 A<br><b>Motor</b> = %4 \u00b0C &nbsp; "
         "<b>Drive</b> = %5 \u00b0C")
          .arg(m_running ? testTimeS() : 0.0, 0, 'f', 1)
          .arg(jt.commanded_current_a, 0, 'f', 2)
          .arg(jt.current_a, 0, 'f', 2)
          .arg(tempText(jt.motor_temp_c), tempText(jt.drive_temp_c)));
}

ThermalTestDialog::WindowStats ThermalTestDialog::computeStats() const {
  WindowStats s;
  if (m_samples.empty())
    return s;
  const double window = windowS();
  s.t_end_s = m_samples.back().t_s;
  if (!m_power.empty())
    s.t_end_s = std::max(s.t_end_s, m_power.back().t_s);
  s.window_start_s = std::max(m_setpoint_t_s, s.t_end_s - window);
  s.span_s = s.t_end_s - s.window_start_s;
  // Allow one temperature sample period of slack.
  s.window_full = s.t_end_s - m_setpoint_t_s >=
                  window - 1.0 / std::max(1.0, m_rate_spin->value());

  LineFit motor{s.window_start_s}, drive{s.window_start_s};
  double sum_cmd = 0.0, sum_act = 0.0;
  const auto first = std::lower_bound(
      m_samples.begin(), m_samples.end(), s.window_start_s,
      [](const Sample &a, double t) { return a.t_s < t; });
  for (auto it = first; it != m_samples.end(); ++it) {
    ++s.temp_samples;
    sum_cmd += it->commanded_current_a;
    sum_act += it->actual_current_a;
    if (it->motor_temp_c >= 0)
      motor.add(it->t_s, it->motor_temp_c);
    if (it->drive_temp_c >= 0)
      drive.add(it->t_s, it->drive_temp_c);
  }
  if (s.temp_samples > 0) {
    s.mean_cmd_a = sum_cmd / s.temp_samples;
    s.mean_actual_a = sum_act / s.temp_samples;
  }
  s.mean_motor_c = motor.mean();
  s.mean_drive_c = drive.mean();
  s.slope_motor_c_min = motor.slopePerMinute();
  s.slope_drive_c_min = drive.slopePerMinute();

  double sv = 0.0, si = 0.0, sp = 0.0, spp = 0.0;
  const auto pfirst = std::lower_bound(
      m_power.begin(), m_power.end(), s.window_start_s,
      [](const PowerSample &a, double t) { return a.t_s < t; });
  for (auto it = pfirst; it != m_power.end(); ++it) {
    ++s.power_samples;
    sv += it->voltage_v;
    si += it->current_a;
    sp += it->power_w;
    spp += it->power_w * it->power_w;
  }
  if (s.power_samples > 0) {
    const double n = s.power_samples;
    s.mean_v = sv / n;
    s.mean_i = si / n;
    s.mean_p = sp / n;
    s.std_p = std::sqrt(std::max(0.0, spp / n - s.mean_p * s.mean_p));
  }

  const double thr = m_ss_threshold_spin->value();
  const auto ok = [thr](const std::optional<double> &slope) {
    return slope && std::fabs(*slope) <= thr;
  };
  bool criterion = false;
  switch (static_cast<SteadySignal>(m_ss_signal_combo->currentIndex())) {
  case SteadySignal::Motor:
    criterion = ok(s.slope_motor_c_min);
    break;
  case SteadySignal::Drive:
    criterion = ok(s.slope_drive_c_min);
    break;
  case SteadySignal::Both:
    criterion = ok(s.slope_motor_c_min) && ok(s.slope_drive_c_min);
    break;
  }
  s.steady = s.window_full && criterion;
  return s;
}

void ThermalTestDialog::refreshStats() {
  if (m_samples.empty()) {
    m_steady = false;
    m_ss_state_label->setText(m_running ? tr("Waiting for samples...")
                                        : tr("Not recording"));
    m_ss_state_label->setStyleSheet(QStringLiteral(
        "font-weight:700; padding:4px; border-radius:4px; "
        "background:#374151; color:#e5e7eb;"));
    m_ss_detail_label->clear();
    return;
  }
  const WindowStats s = computeStats();
  if (m_running) {
    if (s.steady && !m_steady_since_s)
      m_steady_since_s = s.t_end_s;
    else if (!s.steady)
      m_steady_since_s.reset();
    m_steady = s.steady;
  }

  const double window = windowS();
  QString state;
  QString style;
  if (s.steady) {
    state = tr("STEADY STATE (since t = %1 s)")
                .arg(m_steady_since_s.value_or(s.t_end_s), 0, 'f', 0);
    style = QStringLiteral("background:#166534; color:white;");
  } else if (!s.window_full) {
    state = tr("Filling window: %1 / %2 min")
                .arg(std::max(0.0, s.t_end_s - m_setpoint_t_s) / 60.0, 0, 'f', 1)
                .arg(window / 60.0, 0, 'f', 1);
    style = QStringLiteral("background:#374151; color:#e5e7eb;");
  } else {
    state = tr("Not steady (|slope| above %1 \u00b0C/min)")
                .arg(m_ss_threshold_spin->value(), 0, 'f', 3);
    style = QStringLiteral("background:#92400e; color:white;");
  }
  if (!m_running)
    state = tr("Stopped \u2014 last: %1").arg(state);
  m_ss_state_label->setText(state);
  m_ss_state_label->setStyleSheet(
      QStringLiteral("font-weight:700; padding:4px; border-radius:4px; ") +
      style);

  const QString power =
      s.power_samples > 0
          ? tr("%1 W (\u03c3 %2 W) &nbsp; %3 V &nbsp; %4 A &nbsp; [%5 samples]")
                .arg(s.mean_p, 0, 'f', 2)
                .arg(s.std_p, 0, 'f', 2)
                .arg(s.mean_v, 0, 'f', 2)
                .arg(s.mean_i, 0, 'f', 3)
                .arg(s.power_samples)
          : tr("-- (supply not polled)");
  m_ss_detail_label->setText(
      tr("Window: t = %1 \u2026 %2 s (%3 min)<br>"
         "<b>Slope</b> motor %4, drive %5 \u00b0C/min<br>"
         "<b>Mean T</b> motor %6, drive %7 \u00b0C<br>"
         "<b>Mean I</b> cmd %8 A, actual %9 A<br>"
         "<b>Mean supply</b> %10")
          .arg(s.window_start_s, 0, 'f', 0)
          .arg(s.t_end_s, 0, 'f', 0)
          .arg(s.span_s / 60.0, 0, 'f', 1)
          .arg(optText(s.slope_motor_c_min, 3), optText(s.slope_drive_c_min, 3),
               optText(s.mean_motor_c, 2), optText(s.mean_drive_c, 2))
          .arg(s.mean_cmd_a, 0, 'f', 2)
          .arg(s.mean_actual_a, 0, 'f', 2)
          .arg(power));
}

void ThermalTestDialog::updateChartWindows() {
  // Always show the whole test on a shared time axis starting at 0 s.
  const double end = std::max({10.0, m_current_chart->dataMaxX(),
                               m_torque_chart->dataMaxX(),
                               m_temperature_chart->dataMaxX(),
                               m_power_chart->dataMaxX()});
  for (auto *chart : {m_current_chart, m_torque_chart, m_temperature_chart,
                      m_power_chart}) {
    chart->setWindowSeconds(end);
    chart->update();
  }
}

void ThermalTestDialog::togglePowerSupply(bool connect) {
  if (connect) {
    if (m_poller)
      return;
    actuator_test::PowerSupplyConfig cfg;
    cfg.host = m_psu_host_edit->text().trimmed().toStdString();
    try {
      m_poller = std::make_unique<actuator_test::PowerSupplyPoller>(
          cfg, 1.0 / m_psu_rate_spin->value());
    } catch (const std::exception &e) {
      showWarning(tr("DC supply"), QString::fromUtf8(e.what()));
      const QSignalBlocker blocker(m_psu_connect_btn);
      m_psu_connect_btn->setChecked(false);
      return;
    }
    m_psu_max_latency_s = 0.0;
    m_poller->start();
    m_psu_timer->start();
    m_psu_connect_btn->setText(tr("Stop supply polling"));
  } else if (m_poller) {
    m_poller->requestStop();
    m_psu_status_label->setText(tr("Stopping..."));
  }
  updateControls();
}

void ThermalTestDialog::pollPowerSupply() {
  if (!m_poller)
    return;
  const auto samples = m_poller->takeNewSamples();
  bool stored = false;
  for (const auto &s : samples) {
    m_psu_max_latency_s = std::max(m_psu_max_latency_s, s.latency_s);
    if (!m_running || !m_t0_steady_s)
      continue;
    const double t = s.t_s - *m_t0_steady_s;
    if (t < 0.0 || (!m_power.empty() && t <= m_power.back().t_s))
      continue;
    m_power.push_back({t, s.voltage_v, s.current_a, s.power_w, s.latency_s});
    m_power_chart->append(m_series_power, t, s.power_w);
    stored = true;
  }
  if (!samples.empty()) {
    const auto &last = samples.back();
    m_psu_live_label->setText(
        tr("<b>%1 W</b> &nbsp; %2 V &nbsp; %3 A &nbsp; "
           "<span style='color:gray'>latency %4 ms (max %5)</span>")
            .arg(last.power_w, 0, 'f', 2)
            .arg(last.voltage_v, 0, 'f', 3)
            .arg(last.current_a, 0, 'f', 4)
            .arg(last.latency_s * 1e3, 0, 'f', 0)
            .arg(m_psu_max_latency_s * 1e3, 0, 'f', 0));
  }
  if (stored) {
    refreshStats();
    const WindowStats st = computeStats();
    if (st.power_samples > 0)
      m_power_chart->append(m_series_power_mean, m_power.back().t_s,
                            st.mean_p);
    updateChartWindows();
  }

  m_psu_status_label->setText(QString::fromStdString(m_poller->status()));
  if (m_poller->finished()) {
    const QString status = QString::fromStdString(m_poller->status());
    const bool user_stop = !m_psu_connect_btn->isChecked();
    m_psu_auto_started = false;
    m_poller.reset();
    m_psu_timer->stop();
    {
      const QSignalBlocker blocker(m_psu_connect_btn);
      m_psu_connect_btn->setChecked(false);
    }
    m_psu_connect_btn->setText(tr("Start supply polling"));
    if (!user_stop)
      showWarning(tr("DC supply"), status);
    updateControls();
  }
}

void ThermalTestDialog::recordSteadyPoint() {
  if (m_samples.empty())
    return;
  const WindowStats s = computeStats();
  const QString signal = m_ss_signal_combo->currentText();
  m_points.push_back({s, signal});
  const int row = m_points_table->rowCount();
  m_points_table->insertRow(row);
  const auto set = [this, row](int col, const QString &text) {
    m_points_table->setItem(row, col, new QTableWidgetItem(text));
  };
  const bool have_p = s.power_samples > 0;
  set(0, QString::number(row + 1));
  set(1, QString::number(s.t_end_s, 'f', 0));
  set(2, QString::number(s.mean_cmd_a, 'f', 2));
  set(3, QString::number(s.mean_actual_a, 'f', 2));
  set(4, optText(s.mean_motor_c, 2));
  set(5, optText(s.mean_drive_c, 2));
  set(6, optText(s.slope_motor_c_min, 3));
  set(7, optText(s.slope_drive_c_min, 3));
  set(8, have_p ? QString::number(s.mean_v, 'f', 3) : QStringLiteral("--"));
  set(9, have_p ? QString::number(s.mean_i, 'f', 4) : QStringLiteral("--"));
  set(10, have_p ? QString::number(s.mean_p, 'f', 3) : QStringLiteral("--"));
  set(11, have_p ? QString::number(s.std_p, 'f', 3) : QStringLiteral("--"));
  set(12, QString::number(s.span_s, 'f', 0));
  set(13, s.steady ? tr("yes") : tr("no"));
  m_points_table->scrollToBottom();
}

void ThermalTestDialog::showWarning(const QString &title,
                                    const QString &message) {
  auto *box = new QMessageBox(QMessageBox::Warning, title, message,
                              QMessageBox::Ok, this);
  box->setAttribute(Qt::WA_DeleteOnClose);
  box->setWindowModality(Qt::NonModal);
  box->show();
}

void ThermalTestDialog::exportCsv() {
  if (m_running || m_samples.empty())
    return;
  const QString suggested = QDir(m_save_directory).filePath(
      QStringLiteral("thermal-test-%1.csv")
          .arg(QDateTime::currentDateTime().toString(
              QStringLiteral("yyyyMMdd-HHmmss"))));
  const QString path = QFileDialog::getSaveFileName(
      this, tr("Export thermal test"), suggested, tr("CSV file (*.csv)"));
  if (path.isEmpty())
    return;
  const QFileInfo info(path);
  const auto sibling = [&info](const QString &suffix) {
    return info.dir().filePath(info.completeBaseName() + suffix);
  };
  const QString supply_path = sibling(QStringLiteral("-supply.csv"));
  const QString points_path = sibling(QStringLiteral("-points.csv"));
  const QString profiles_path = sibling(QStringLiteral("-profiles.csv"));
  QStringList existing;
  if (!m_power.empty() && QFile::exists(supply_path))
    existing << supply_path;
  if (!m_points.empty() && QFile::exists(points_path))
    existing << points_path;
  if (!m_profiles.empty() && QFile::exists(profiles_path))
    existing << profiles_path;
  if (!existing.isEmpty() &&
      QMessageBox::question(this, tr("Export"),
                            tr("Replace existing files?\n%1")
                                .arg(existing.join(QLatin1Char('\n')))) !=
          QMessageBox::Yes)
    return;

  saveCsv(path);
}

void ThermalTestDialog::autoSaveCsv() {
  if (m_samples.empty())
    return;
  const QString &base = m_save_directory;
  const QString directory =
      QDir(base).filePath(QStringLiteral("%1-%2")
                             .arg(QDateTime::currentDateTimeUtc().toString(
                                      QStringLiteral("yyyyMMdd-HHmmss-zzz")),
                                  QUuid::createUuid().toString(
                                      QUuid::WithoutBraces)));
  if (base.isEmpty() || !QDir().mkpath(directory)) {
    m_status_label->setText(
        m_status_label->text() + tr("\nAutomatic CSV save failed."));
    showWarning(tr("Automatic CSV save"),
                tr("Could not create the automatic save directory: %1. "
                   "Data remains in memory; use Export CSV before starting "
                   "another recording.")
                    .arg(directory));
    return;
  }
  const QString path =
      QDir(directory).filePath(QStringLiteral("thermal-test.csv"));
  if (saveCsv(path))
    m_status_label->setText(m_status_label->text() +
                           tr("\nCSV saved automatically to %1").arg(directory));
  else
    m_status_label->setText(
        m_status_label->text() +
        tr("\nAutomatic CSV save failed. Use Export CSV before starting "
           "another recording."));
}

bool ThermalTestDialog::saveCsv(const QString &path) {
  const QFileInfo info(path);
  const QString supply_path =
      info.dir().filePath(info.completeBaseName() + QStringLiteral("-supply.csv"));
  const QString points_path =
      info.dir().filePath(info.completeBaseName() + QStringLiteral("-points.csv"));
  const QString profiles_path =
      info.dir().filePath(info.completeBaseName() + QStringLiteral("-profiles.csv"));
  const auto write = [this](const QString &file_path, auto &&body) {
    QSaveFile file(file_path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
      showWarning(tr("Export"), tr("Could not write %1: %2")
                                    .arg(file_path, file.errorString()));
      return false;
    }
    QTextStream out(&file);
    out.setRealNumberPrecision(10);
    body(out);
    out.flush();
    if (out.status() != QTextStream::Ok || !file.commit()) {
      showWarning(tr("Export"), tr("Could not save %1: %2")
                                    .arg(file_path, file.errorString()));
      return false;
    }
    return true;
  };

  if (!write(path, [this](QTextStream &out) {
        out << "time_s,commanded_current_a,actual_current_a,motor_temp_c,"
               "drive_temp_c,steady,cyclic_active,commanded_i2_a2s,actual_i2_a2s,"
               "pt1000_voltage_v,pt1000_temp_c,pt1000_steady_time_s,"
               "torsiometer_raw_nm,torsiometer_filtered_nm,torsiometer_steady_time_s,"
               "torsiometer_raw_mean_nm,torsiometer_mean_window_s,"
               "torsiometer_mean_span_s,torsiometer_mean_samples\n";
        for (const auto &s : m_samples) {
          out << s.t_s << ',' << s.commanded_current_a << ','
              << s.actual_current_a << ',' << s.motor_temp_c << ','
              << s.drive_temp_c << ',' << (s.steady ? 1 : 0) << ','
              << (s.cyclic_active ? 1 : 0) << ','
              << s.commanded_i2_a2s << ',' << s.actual_i2_a2s << ',';
          if (s.pt1000)
            out << s.pt1000->voltage_v << ',' << s.pt1000->temperature_c
                << ',' << s.pt1000->steady_time_s;
          else
            out << ",,";
          out << ',';
          if (s.torsiometer)
            out << s.torsiometer->torque_nm << ','
                << s.torsiometer->filtered_torque_nm << ','
                << s.torsiometer->steady_time_s;
          else
            out << ",,";
          out << ',';
          if (s.torque_mean)
            out << s.torque_mean->torque_nm;
          out << ',' << s.torque_mean_window_s << ',';
          if (s.torque_mean)
            out << s.torque_mean->span_s << ',' << s.torque_mean->samples;
          else
            out << ',';
          out << '\n';
        }
      }))
    return false;
  if (!m_power.empty() &&
      !write(supply_path, [this](QTextStream &out) {
        out << "time_s,supply_voltage_v,supply_current_a,supply_power_w,"
               "query_latency_s\n";
        for (const auto &s : m_power)
          out << s.t_s << ',' << s.voltage_v << ',' << s.current_a << ','
              << s.power_w << ',' << s.latency_s << '\n';
      }))
    return false;
  if (!m_profiles.empty() &&
      !write(profiles_path, [this](QTextStream &out) {
        out << "requested_time_s,observed_start_time_s,end_time_s,kt_output_nm_per_a,"
               "stance_current_a,swing_current_a,period_s,stance_fraction,"
               "transition_s,entry_exit_s,cycling_duration_s,theoretical_cycle_rms_a\n";
        for (const auto &run : m_profiles) {
          const auto &p = run.profile;
          out << run.requested_s << ',';
          if (run.started_s)
            out << *run.started_s;
          out << ',';
          if (run.ended_s)
            out << *run.ended_s;
          out << ',' << run.kt_nm_a << ',' << p.stance_a << ',' << p.swing_a
              << ',' << p.period_s << ',' << p.stance_fraction << ','
              << p.transition_s << ',' << p.entry_exit_s << ',' << p.duration_s
              << ',' << p.cycleRmsA() << '\n';
        }
      }))
    return false;
  if (!m_points.empty()) {
    return write(points_path, [this](QTextStream &out) {
      const auto opt = [](const std::optional<double> &v) {
        return v ? QString::number(*v, 'g', 10) : QString();
      };
      out << "point,time_s,window_start_s,window_s,commanded_current_a,"
             "actual_current_a,motor_temp_c,drive_temp_c,"
             "motor_slope_c_per_min,drive_slope_c_per_min,supply_voltage_v,"
             "supply_current_a,supply_power_w,supply_power_std_w,"
             "supply_samples,steady,criterion\n";
      for (std::size_t i = 0; i < m_points.size(); ++i) {
        const auto &s = m_points[i].stats;
        const bool have_p = s.power_samples > 0;
        out << (i + 1) << ',' << s.t_end_s << ',' << s.window_start_s << ','
            << s.span_s << ',' << s.mean_cmd_a << ',' << s.mean_actual_a
            << ',' << opt(s.mean_motor_c) << ',' << opt(s.mean_drive_c) << ','
            << opt(s.slope_motor_c_min) << ',' << opt(s.slope_drive_c_min)
            << ',';
        if (have_p)
          out << s.mean_v << ',' << s.mean_i << ',' << s.mean_p << ','
              << s.std_p;
        else
          out << ",,,";
        out << ',' << s.power_samples << ',' << (s.steady ? 1 : 0) << ",\""
            << m_points[i].signal << "\"\n";
      }
    });
  }
  return true;
}

void ThermalTestDialog::closeEvent(QCloseEvent *event) {
  if (m_running) {
    emit currentSetpointRequested(m_active_joint, 0.0, /*release=*/true);
    finishRecording(tr("window closed"));
  }
  if (m_poller) {
    // Polling only makes sense with the view open; stop it to free the
    // instrument connection.
    m_psu_connect_btn->setChecked(false);
  }
  m_temperature_connect->setChecked(false);
  event->accept();
}

} // namespace actuator_test::gui
