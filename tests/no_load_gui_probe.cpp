// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QElapsedTimer>
#include <QFile>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTableWidget>
#include <QTemporaryDir>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

#include "core/controller_worker.hpp"
#include "widgets/plot_panel.hpp"

// Simulate reader publication without opening NI tasks, sockets or the bus.
#define private public
#include "widgets/no_load_test_panel.hpp"
#include "widgets/locked_rotor_test_panel.hpp"
#include "widgets/damped_current_settle_panel.hpp"
#undef private

#include <cassert>
#include <iostream>
#include <limits>

using namespace actuator_test;
using namespace actuator_test::gui;

class Probe : public NoLoadTestDialog {
public:
  int triggers = 0;
  int releases = 0;
  int requested_kp = 0, requested_kd = 0;
  int zero_hold_commands = 0;
  double expected_speed = 60;
  std::shared_ptr<ExternalDaqReader> reader =
      std::make_shared<ExternalDaqReader>();
  TelemetryFrame frame;

  Probe()
      : NoLoadTestDialog(nullptr, QStandardPaths::writableLocation(
                                      QStandardPaths::AppLocalDataLocation)) {
    assert(!m_continuous->isChecked());
    assert(!m_start->isEnabled());
    JointInfo joint;
    joint.pvt_kd = 50;
    setJoints({joint});
    assert(m_kp->value() == 0 && m_kd->value() == 50);
    setAvailable(true);
    setDriveCapture(std::make_shared<DriveCapture>(), 0.01);
    frame.t_s = 1;
    frame.state = ControllerState::Connected;
    frame.joints.resize(1);
    appendTelemetry(frame);
    tick();
    assert(!m_start->isEnabled());
    reader->m_running = true;
    reader->m_latest = {1, 0.1, 0.1, 10};
    setExternalDaqReader(reader);
    tick();
    assert(m_start->isEnabled());
    connect(this, &NoLoadTestDialog::speedHoldRequested, this,
            [this](std::size_t joint, double speed, double, bool continuous,
                   bool release, int32_t kp, int32_t kd) {
              assert(joint == 0 && !continuous);
              if (!release && speed == 0)
                ++zero_hold_commands;
              else
                assert(speed == (release ? 0 : expected_speed));
              assert(kp == m_kp->value() && kd == m_kd->value());
              requested_kp = kp;
              requested_kd = kd;
              if (release)
                ++releases;
            });
    m_dwell->setValue(0.1);
    start();
    assert(isRunning() && !m_settings->isEnabled());
    frame.state = ControllerState::Jogging;
    frame.joints[0].speed_hold_active = true;
    frame.joints[0].speed_hold_target_deg_s = 60;
    frame.joints[0].speed_hold_kd = 50;
    frame.joints[0].velocity_deg_s = 60;
    publish();
  }

  ~Probe() override {
    stopTest("probe cleanup");
    reader->m_running = false;
  }

  void publish(double torque = 0.1, double speed = 10) {
    frame.t_s += 1;
    appendTelemetry(frame);
    reader->m_latest = {reader->m_latest.t_s + 1, torque, torque, speed};
    reader->captureBuffer().push(reader->m_latest);
    m_drive_capture->push({0, static_cast<uint64_t>(frame.t_s * 1000),
                           frame.t_s, frame.t_s + 0.001,
                           frame.joints[0].velocity_deg_s / 6.0});
    tick();
  }

  void qualify() {
    assert(triggers == 0 && m_dwell_start >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    publish();
    assert(triggers == 0 && m_acquire->isEnabled());
    m_acquire->click();
    assert(triggers == 1 && isRunning());
  }

private:
  void beginAcquisition() override {
    ++triggers;
    m_phase = Phase::Capturing;
    m_dataset_pending = true;
    m_cancelled = m_supply_done = m_supply_ok = false;
    m_acquisition_complete = false;
    m_sensor_samples.clear();
    m_supply_samples.clear();
    m_session = "no-load-regression";
    m_metadata = "# source=hardware-free regression\n";
    // No PowerSupplyReader is created or started.
  }
};

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  assert(argc == 2);
  const std::string test = argv[1];
  Probe p;
  if (test == "damped-settle") {
    DampedCurrentSettleDialog dialog;
    JointInfo joint;
    joint.rated_current_a = 2;
    dialog.setJoints({joint});
    dialog.setExternalDaqReader(p.reader);
    dialog.m_confirm_check->setChecked(true);
    dialog.m_duration_spin->setValue(0.1);
    dialog.m_settle_spin->setValue(0.1);
    int starts = 0, releases = 0;
    QObject::connect(
        &dialog, &DampedCurrentSettleDialog::dampedCurrentRequested, &dialog,
        [&](std::size_t index, double, double, double, double, bool release) {
          assert(index == 0);
          release ? ++releases : ++starts;
        });
    const auto run = [&](double origin) {
      dialog.start();
      assert(dialog.isRunning() && !dialog.m_joint_combo->isEnabled() &&
             !dialog.m_start_btn->isEnabled() && dialog.m_stop_btn->isEnabled());
      TelemetryFrame frame;
      frame.joints.resize(1);
      frame.t_s = origin;
      dialog.appendTelemetry(frame);
      assert(dialog.m_t0_s == origin);
      p.reader->m_latest.t_s += 1;
      p.reader->m_latest.torque_nm = 2;
      frame.t_s = origin + 0.125;
      dialog.appendTelemetry(frame);
      dialog.appendTelemetry(frame);
      assert(dialog.m_torque_samples == 1);
      p.reader->m_latest.t_s += 1;
      p.reader->m_latest.torque_nm = 4;
      frame.t_s = origin + 0.25;
      dialog.appendTelemetry(frame);
      assert(!dialog.isRunning() && dialog.m_torque_samples == 2);
      assert(dialog.m_measured_offset_nm == 3);
      assert(dialog.m_joint_combo->isEnabled() &&
             dialog.m_start_btn->isEnabled() && !dialog.m_stop_btn->isEnabled());
    };
    run(0);
    dialog.tare();
    assert(p.reader->tareOffsetNm() == 3 && !dialog.m_tare_btn->isEnabled());
    run(10);
    assert(starts == 2 && releases == 2);
  } else if (test == "locked-rotor-sweep") {
    LockedRotorTestDialog dialog;
    dialog.setJoints({JointInfo{}});
    dialog.m_start_spin->setValue(0);
    dialog.m_end_spin->setValue(1);
    dialog.m_step_spin->setValue(0.6);
    dialog.m_opposite_sign_check->setChecked(true);
    dialog.buildSteps();
    assert((dialog.m_steps_a ==
            std::vector<double>{0, 0.6, 1, 0.6, 0, 0, -0.6, -1, -0.6, 0}));
    assert(dialog.m_forward_step_count == 3);
    dialog.m_negative_only_check->setChecked(true);
    dialog.buildSteps();
    assert((dialog.m_steps_a == std::vector<double>{0, -0.6, -1, -0.6, 0}));
    assert(!dialog.m_opposite_sign_check->isEnabled());
    dialog.m_end_spin->setValue(0);
    dialog.buildSteps();
    assert((dialog.m_steps_a == std::vector<double>{0}));

    dialog.m_negative_only_check->setChecked(false);
    dialog.m_opposite_sign_check->setChecked(false);
    dialog.m_return_sweep_check->setChecked(false);
    dialog.m_end_spin->setValue(0.5);
    dialog.m_step_spin->setValue(0.5);
    dialog.m_dwell_spin->setValue(0.2);
    dialog.m_sweep_count_spin->setValue(2);
    dialog.m_confirm_check->setChecked(true);
    std::vector<double> targets;
    int releases = 0;
    QObject::connect(
        &dialog, &LockedRotorTestDialog::currentSetpointRequested, &dialog,
        [&](std::size_t joint, double current, bool release) {
          assert(joint == 0);
          if (release)
            ++releases;
          else
            targets.push_back(current);
        });
    dialog.onStartClicked();
    assert(dialog.isBusy() && !dialog.m_joint_combo->isEnabled());
    assert(!dialog.m_psu_host_edit->isEnabled());
    assert(!dialog.m_export_btn->isEnabled());
    TelemetryFrame frame;
    frame.joints.resize(1);
    for (const double t : {0.0, 0.25, 1.0, 1.25, 2.0, 2.25, 3.0, 3.25}) {
      frame.t_s = t;
      dialog.appendTelemetry(frame);
    }
    assert((targets == std::vector<double>{0, 0.5, 0, 0.5}));
    assert(releases == 1 && !dialog.isBusy());
    assert(dialog.m_results_table->rowCount() == 4);
    assert(dialog.m_raw_samples.front().step_elapsed_s == 0);
    assert(dialog.m_joint_combo->isEnabled() && dialog.m_export_btn->isEnabled());
    dialog.m_psu = std::make_unique<PowerSupplyReader>();
    dialog.updateControls();
    assert(!dialog.m_start_btn->isEnabled() &&
           !dialog.m_psu_host_edit->isEnabled() &&
           !dialog.m_export_btn->isEnabled());
    dialog.m_psu.reset();
    dialog.updateControls();
    assert(dialog.m_start_btn->isEnabled() && dialog.m_psu_host_edit->isEnabled());
  } else if (test == "locked-rotor-ramp") {
    LockedRotorTestDialog dialog;
    dialog.setJoints({JointInfo{}});
    dialog.m_start_spin->setValue(0);
    dialog.m_end_spin->setValue(0);
    dialog.m_dwell_spin->setValue(0.2);
    dialog.m_smooth_transition_check->setChecked(true);
    assert(dialog.m_ramp_time_spin->isEnabled());
    dialog.onStartClicked();
    assert(!dialog.m_ramp_time_spin->isEnabled());
    TelemetryFrame frame;
    frame.joints.resize(1);
    frame.joints[0].current_transition_active = true;
    dialog.appendTelemetry(frame);
    assert(!dialog.m_step_t0_s && dialog.m_sample_count == 0);
    frame.t_s = 1;
    frame.joints[0].current_transition_active = false;
    dialog.appendTelemetry(frame);
    assert(dialog.m_step_t0_s == 1 && dialog.m_sample_count == 1);
    assert(dialog.m_raw_samples.back().step_elapsed_s == 1);
    frame.t_s = 1.25;
    dialog.appendTelemetry(frame);
    assert(!dialog.isRunning() && dialog.m_ramp_time_spin->isEnabled());
    assert(dialog.m_results_table->rowCount() == 1);
  } else if (test == "dataset-export") {
    QTemporaryDir directory;
    assert(directory.isValid());
    const std::vector<ExternalDaqReader::Sample> sensor{{1, 2, 3, 4, 6, 5}};
    const std::vector<PowerSupplyReader::Sample> supply{{7, 8, 9, 72}};
    const std::vector<DriveSample> drive{{1, 11, 10, 12, 13},
                                         {0, 99, 98, 97, 96}};
    const NoLoadDatasetView dataset{"# session=export-test\n",
                                    "first\nsecond",
                                    true,
                                    1,
                                    2,
                                    3,
                                    2,
                                    4,
                                    5,
                                    sensor,
                                    supply,
                                    drive};
    const QByteArray metadata = "# session=export-test\n"
                                "# interrupted=1\n"
                                "# daq_dropped_samples=2\n"
                                "# drive_dropped_samples=3\n"
                                "# drive_late_interval_threshold_s=3\n"
                                "# drive_late_intervals=4\n"
                                "# drive_max_host_interval_s=5\n"
                                "# result=first\n# second\n"
                                "# synchronization=independent clocks; "
                                "synchronize in post-processing\n";
    const auto check = [&](NoLoadCsvSource source, const QByteArray &rows) {
      QString error = "previous error";
      const QString path = directory.filePath("dataset.csv");
      assert(saveNoLoadCsv(path, source, dataset, error));
      assert(error.isEmpty());
      QFile file(path);
      assert(file.open(QIODevice::ReadOnly));
      assert(file.readAll() == metadata + rows);
    };
    check(NoLoadCsvSource::Sensor,
          "# source=buffered DAQ block results; 100 Hz; not raw 50 kHz stream\n"
          "# daq_time_s=hardware sample count / analog sampling rate\n"
          "daq_time_s,external_raw_torque_nm,external_filtered_torque_nm,"
          "external_speed_rpm,sample_index,delivery_time_s\n"
          "1,2,3,4,5,6\n");
    check(NoLoadCsvSource::Supply,
          "# source=instrument ELOG samples\n"
          "elog_time_s,supply_voltage_v,supply_current_a\n7,8,9\n");
    check(
        NoLoadCsvSource::Drive,
        "# source=buffered EtherCAT telemetry; nominal 100 Hz\n"
        "# host_time_s=host steady clock at telemetry publication; "
        "cycle time is nominal, inspect host intervals for jitter\n"
        "drive_time_s,drive_speed_rpm,cycle_index,host_time_s\n10,13,11,12\n");
    QString error;
    assert(!saveNoLoadCsv(directory.filePath("missing/dataset.csv"),
                          NoLoadCsvSource::Sensor, dataset, error));
    assert(!error.isEmpty());
  } else if (test == "supply-completed-between-polls") {
    p.qualify();
    p.m_phase = Probe::Phase::Capturing;
    p.m_supply = std::make_unique<PowerSupplyReader>();
    p.m_supply->setProgress(PowerSupplyReader::State::Acquiring,
                            "display wording is not a protocol");
    p.m_supply->setProgress(PowerSupplyReader::State::Complete,
                            "download finished between GUI polls");
    p.m_supply->m_samples = {{0, 48, 1, 48}};
    p.m_supply->m_finished = true;
    p.publish();
    assert(p.m_acquisition_complete && !p.m_cancelled);
    assert(p.isRunning() && p.releases == 0);
  } else if (test == "torque-mean") {
    p.resetTorqueMean();
    p.updateTorqueReadout({1, 1, 1, 10}, 0);
    p.updateTorqueReadout({2, 3, 3, 10}, 0.25);
    assert(p.m_live->text().contains("mean (last 100 samples): --"));
    p.updateTorqueReadout({3, 2, 2, 10}, 0.5);
    assert(p.m_live->text().contains("3/100 samples): 2.000 Nm"));
    p.updateTorqueReadout({4, 8, 8, 10}, 0.75);
    assert(p.m_live->text().contains("3/100 samples): 2.000 Nm"));
    p.updateTorqueReadout({5, 4, 4, 10}, 1.0);
    assert(p.m_live->text().contains("5/100 samples): 3.600 Nm"));
    p.resetTorqueMean();
    for (int i = 0; i < 150; ++i)
      p.updateTorqueReadout({double(i + 1), double(i), double(i), 10},
                            i * 0.025);
    assert(p.m_torque_window.size() == 100);
    assert(p.m_torque_window.front() == 50);
    assert(p.m_torque_sum == 9950);
    p.updateTorqueReadout({151, 150, 150, 10}, 4.0);
    assert(p.m_live->text().contains("100/100 samples): 100.500 Nm"));
    p.resetTorqueMean();
    p.updateTorqueReadout({6, 0, 0, 10}, 2.0);
    assert(p.m_live->text().contains("mean (last 100 samples): --"));
  } else if (test == "plots") {
    p.qualify();
    assert(p.m_speed_chart->dataMaxX() > 0.1);
    assert(p.m_torque_chart->dataMaxX() > 0.1);
    assert(p.m_power_chart->dataMaxX() == 0.0);
    p.publish();
    p.m_supply_samples = {{0, 48, 1, 48}, {0.5, 48, 2, 96}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_acquisition_complete);
    assert(p.zero_hold_commands == 0 && p.releases == 0 && p.isRunning());
    assert(p.m_target_rpm == 10 && !p.m_speed_change_pending);
    assert(p.m_power_chart->dataMaxX() == 0.5);
    assert(p.m_power_label->text().contains("mean 72.00 W"));
    assert(p.m_sensor_path.startsWith(
        QDir(p.m_result_root).filePath("no_load/no-load-regression/")));
    assert(QFile::exists(p.m_sensor_path));
    assert(QFile::exists(p.m_supply_path));
  } else if (test == "gain-edit") {
    p.stopTest("edit gains");
    p.frame.state = ControllerState::Connected;
    p.frame.joints[0].speed_hold_active = false;
    p.appendTelemetry(p.frame);
    p.m_kd->setValue(0);
    assert(!p.m_start->isEnabled());
    p.m_kp->setValue(123);
    assert(p.m_start->isEnabled());
    p.m_kd->setValue(456);
    p.start();
    assert(p.isRunning() && p.requested_kp == 123 && p.requested_kd == 456);
    assert(!p.m_settings->isEnabled());
    assert(p.m_joints[0].pvt_kp == 0 && p.m_joints[0].pvt_kd == 50);
    p.frame.state = ControllerState::Jogging;
    p.frame.joints[0].speed_hold_active = true;
    p.frame.joints[0].speed_hold_kp = 123;
    p.frame.joints[0].speed_hold_kd = 456;
    p.publish();
    assert(p.isRunning() && p.m_motion_ack);
  } else if (test == "gain-selection") {
    p.stopTest("select joint");
    JointInfo first, second;
    first.pvt_kp = 20000;
    second.pvt_kp = 111;
    second.pvt_kd = 222;
    p.setJoints({first, second});
    assert(p.m_kp->value() == 20000 && p.m_kd->value() == 0);
    p.m_kp->setValue(999);
    p.m_joint->setCurrentIndex(1);
    assert(p.m_kp->value() == 111 && p.m_kd->value() == 222);
    assert(p.m_joints[0].pvt_kp == 20000);
  } else if (test == "gain-mismatch") {
    p.frame.joints[0].speed_hold_kd = 51;
    p.publish();
    assert(!p.isRunning() && p.releases == 1);
  } else if (test == "gain-metadata") {
    p.buildMetadata();
    assert(p.m_metadata.contains(
        "# band_check=advisory; torsiometer torque and speed only\n"));
    assert(p.m_metadata.contains("# pvt_kp=0\n"));
    assert(p.m_metadata.contains("# pvt_kd=50\n"));
    p.m_kp->setValue(123);
    p.m_kd->setValue(456);
    p.buildMetadata();
    assert(p.m_metadata.contains("# pvt_kp=123\n"));
    assert(p.m_metadata.contains("# pvt_kd=456\n"));
  } else if (test == "dwell-once") {
    p.qualify();
    for (int n = 0; n < 10; ++n)
      p.publish();
    assert(p.triggers == 1 && !p.m_rearm->isEnabled());
  } else if (test == "out-of-band-reset") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish(-1);
    assert(p.triggers == 0 && p.m_dwell_start < 0);
    p.publish();
    assert(p.triggers == 0 && p.m_dwell_start >= 0);
    p.qualify();
  } else if (test == "duplicate") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.tick();
    assert(p.triggers == 0);
    p.publish();
    assert(p.triggers == 0 && p.m_acquire->isEnabled());
    p.m_acquire->click();
    assert(p.triggers == 1);
    p.publish();
    const auto count = p.m_sensor_samples.size();
    p.tick();
    assert(p.m_sensor_samples.size() == count);
  } else if (test == "stale") {
    p.m_dwell_start = p.now() - 0.2;
    p.m_last_fresh_wall = p.now() - 1;
    p.tick();
    assert(p.triggers == 0 && !p.isRunning() && p.releases == 1);
  } else if (test == "stale-newer") {
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    p.publish();
    assert(p.triggers == 0 && !p.isRunning() && p.releases == 1);
  } else if (test == "invalid-sample") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish(std::numeric_limits<double>::quiet_NaN());
    assert(p.triggers == 0 && !p.isRunning());
  } else if (test == "manual-current") {
    assert(!p.m_current_controls->isEnabled());
    p.stopTest("current adjustment test");
    JointInfo joint;
    joint.driver_name = "MT_Device";
    joint.selectable = true;
    joint.rated_current_a = 2;
    joint.pvt_kd = 50;
    p.setJoints({joint});
    p.frame.state = ControllerState::Connected;
    p.frame.joints[0].speed_hold_active = false;
    p.frame.joints[0].min_limit_deg = -180;
    p.frame.joints[0].max_limit_deg = 180;
    p.publish();
    assert(p.m_current_controls->isEnabled());
    int commands = 0;
    double requested = 0, ramp = 0;
    QObject::connect(&p, &NoLoadTestDialog::currentRampRequested,
                     [&](std::size_t j, double a, double r) {
                       assert(j == 0);
                       ++commands;
                       requested = a;
                       ramp = r;
                     });
    p.m_current_ramp->setValue(0.5);
    assert(p.m_current_step->minimum() == 0.002);
    assert(p.m_current->singleStep() == 0.002);
    p.m_current_step->setValue(0.01);
    p.m_current_plus->click();
    assert(commands == 1 && requested == 0.01 && ramp == 0.5);
    p.m_current_step->setValue(0.02);
    assert(commands == 1);
    p.m_current_minus->click();
    assert(commands == 2 && std::fabs(requested + 0.01) < 1e-12);
    p.m_current->setValue(10);
    assert(requested == 2 && !p.m_current_plus->isEnabled());
    p.frame.state = ControllerState::Jogging;
    p.frame.joints[0].current_control_active = true;
    p.frame.joints[0].commanded_current_a = 2;
    p.frame.joints[0].current_a = 1.9;
    p.publish();
    assert(p.m_current_readout->text().contains("reported: 1.900000 A"));
    p.m_acquire->click();
    assert(p.isBusy() && p.m_current_controls->isEnabled());
    p.m_current_minus->click();
    assert(std::fabs(requested - 1.98) < 1e-12);
    p.publish();
    p.m_soft_stop->click();
    assert(requested == 0 && ramp == 0.5 && p.m_cancelled);
    assert(p.m_current->value() == 0);
    p.setAvailable(false);
    const int before = commands;
    p.m_current->setValue(0.5);
    assert(commands == before);
    p.setAvailable(true);
    p.frame.state = ControllerState::Faulted;
    p.publish();
    assert(!p.m_current_controls->isEnabled());
    joint.rated_current_a = 13.4;
    p.setJoints({joint});
    assert(commands == before);
    assert(std::fabs(p.m_current_step->minimum() - 0.0134) < 1e-12);
    assert(std::fabs(p.m_current->singleStep() - 0.0134) < 1e-12);
    p.frame.state = ControllerState::Connected;
    p.frame.joints[0].commanded_current_a = 0;
    p.publish();
    p.setJoints({joint});
    assert(p.m_current->value() == 0);
    p.m_current_plus->click();
    assert(std::fabs(requested - 0.0134) < 1e-12);
    assert(std::lround(requested / 13.4 * 1000) == 1);
    p.m_current->setValue(-0.02);
    assert(std::fabs(requested + 0.0134) < 1e-12);
    p.m_current_step->setValue(0.03);
    assert(std::fabs(p.m_current_step->value() - 0.0268) < 1e-12);
    p.m_current_minus->click();
    assert(std::fabs(requested + 0.0402) < 1e-12);
  } else if (test == "independent-acquisition") {
    p.stopTest("switch to external motion control");
    const int releases = p.releases;
    p.frame.joints[0].speed_hold_active = false;
    p.frame.state = ControllerState::Jogging;
    p.m_kp->setValue(0);
    p.m_kd->setValue(0);
    p.m_rpm->setValue(0);
    p.publish();
    assert(!p.isRunning() && p.m_acquire->isEnabled());
    p.m_acquire->click();
    assert(p.triggers == 1 && p.isBusy() && !p.isRunning());
    assert(!p.m_acquire->isEnabled() && !p.m_settings->isEnabled());
    assert(!p.m_soft_stop->isEnabled() && p.m_stop->isEnabled());
    p.frame.joints[0].velocity_deg_s = 120;
    p.publish(0.1, 20);
    p.buildMetadata();
    assert(p.m_metadata.contains("# dialog_gains_applied=0"));
    assert(
        p.m_metadata.contains("# dut_motion_control=external to this dialog"));
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_acquisition_complete && !p.isBusy());
    assert(p.releases == releases && p.zero_hold_commands == 0);
    assert(p.m_acquire->isEnabled() && p.m_export->isEnabled());
    p.m_acquire->click();
    assert(p.triggers == 2 && p.isBusy());
    p.frame.state = ControllerState::Faulted;
    p.publish();
    assert(p.m_cancelled && p.releases == releases);
  } else if (test == "independent-emergency") {
    p.stopTest("external control");
    p.frame.joints[0].speed_hold_active = false;
    p.publish();
    int emergencies = 0;
    QObject::connect(&p, &NoLoadTestDialog::emergencyStopRequested,
                     [&] { ++emergencies; });
    p.m_acquire->click();
    p.m_stop->click();
    assert(emergencies == 1 && p.m_cancelled);
  } else if (test == "independent-unavailable") {
    p.stopTest("external control");
    p.frame.joints[0].speed_hold_active = false;
    p.publish();
    p.setAvailable(false);
    assert(!p.m_acquire->isEnabled());
    p.startAcquisition();
    assert(p.triggers == 0);
    p.setAvailable(true);
    p.m_host->clear();
    p.updateControls();
    assert(!p.m_acquire->isEnabled());
  } else if (test == "buffered-100hz") {
    p.qualify();
    // Ten producer updates occur while the GUI does not poll either source.
    for (int i = 1; i <= 10; ++i) {
      p.reader->captureBuffer().push({2 + i * 0.01, i * 0.1, i * 0.05, 10,
                                      2 + i * 0.01 + 0.003,
                                      static_cast<uint64_t>(100000 + i * 500)});
      p.m_drive_capture->push({0, static_cast<uint64_t>(2000 + i * 10),
                               2 + i * 0.01, 100 + i * 0.01, i * 1.0});
      p.m_drive_capture->push({1, 0, 0, 0, 999});
    }
    p.tick();
    assert(p.m_sensor_samples.size() == 10 && p.m_drive_samples.size() == 10);
    for (int i = 1; i < 10; ++i) {
      assert(std::fabs(p.m_sensor_samples[i].t_s -
                       p.m_sensor_samples[i - 1].t_s - 0.01) < 1e-12);
      assert(p.m_sensor_samples[i].sample_index -
                 p.m_sensor_samples[i - 1].sample_index ==
             500);
      assert(p.m_drive_samples[i].cycle_index -
                 p.m_drive_samples[i - 1].cycle_index ==
             10);
    }
    p.tick();
    assert(p.m_sensor_samples.size() == 10 && p.m_drive_samples.size() == 10);
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_acquisition_complete && p.m_drive_late_intervals == 0);
    QFile csv(p.m_sensor_path);
    assert(csv.open(QIODevice::ReadOnly));
    int rows = 0;
    for (const auto &line : QString::fromUtf8(csv.readAll()).split('\n')) {
      if (line.startsWith('#') || line.isEmpty())
        continue;
      const auto fields = line.split(',');
      assert(fields.size() == 6);
      if (line.startsWith("daq_time_s")) {
        assert(fields[4] == "sample_index" && fields[5] == "delivery_time_s");
        continue;
      }
      const auto &sample = p.m_sensor_samples.at(rows++);
      assert(fields[0].toDouble() == sample.t_s);
      assert(fields[4].toULongLong() == sample.sample_index);
      assert(fields[5].toDouble() == sample.delivery_time_s);
    }
    assert(rows == 10);
    p.stopTest("buffered test stop");
    p.reader->captureBuffer().push({10, 0, 0, 10});
    p.drainCapture();
    assert(p.m_sensor_samples.size() == 10);
  } else if (test == "capture-overflow") {
    p.setDriveCapture(std::make_shared<DriveCapture>(2), 0.01);
    // setDriveCapture stops motion; restore the simulated acknowledged hold.
    p.m_running = p.m_motion_ack = true;
    p.m_phase = Probe::Phase::Dwell;
    p.m_dwell_start = p.now();
    p.qualify();
    for (int i = 0; i < 3; ++i)
      p.m_drive_capture->push({0, static_cast<uint64_t>(i), i * 0.01, 0, 10});
    p.tick();
    assert(p.m_cancelled && !p.isRunning() && p.m_drive_dropped == 1);
    assert(p.m_result.contains("buffer overflow"));
    p.finishAcquisition();
    assert(!p.m_acquisition_complete && p.m_export->isEnabled());
    QFile csv(p.m_drive_path);
    assert(csv.open(QIODevice::ReadOnly));
    assert(csv.readAll().contains("# drive_dropped_samples=1"));
  } else if (test == "capture-rate") {
    p.m_drive_period_s = 0.02;
    p.updateControls();
    assert(!p.m_acquire->isEnabled());
    p.startAcquisition();
    assert(p.triggers == 0);
  } else if (test == "drive-speed-csv") {
    p.qualify();
    p.frame.joints[0].velocity_deg_s = 123;
    p.publish(0.1, 19);
    const double first_drive_time = p.frame.t_s;
    p.frame.joints[0].velocity_deg_s = -42;
    p.publish(0.2, -6);
    assert(p.m_sensor_samples.size() == 2);
    assert(p.m_drive_samples[0].speed_rpm == 20.5);
    assert(p.m_drive_samples[0].t_s == first_drive_time);
    assert(p.m_drive_samples[1].speed_rpm == -7);
    p.m_drive_samples[1].host_time_s = p.m_drive_samples[0].host_time_s + 0.02;
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_drive_late_intervals == 1);
    assert(p.m_result.contains("Timing warning"));
    QFile csv(p.m_drive_path);
    assert(csv.open(QIODevice::ReadOnly));
    int rows = 0;
    for (const auto &line : QString::fromUtf8(csv.readAll()).split('\n')) {
      if (line.startsWith('#') || line.isEmpty())
        continue;
      const auto fields = line.split(',');
      assert(fields.size() == 4);
      if (line.startsWith("drive_time_s")) {
        assert(fields[1] == "drive_speed_rpm" && fields[2] == "cycle_index");
        continue;
      }
      const auto &sample = p.m_drive_samples.at(rows++);
      assert(fields[0].toDouble() == sample.t_s);
      assert(fields[1].toDouble() == sample.speed_rpm);
      assert(fields[2].toULongLong() == sample.cycle_index);
      assert(fields[3].toDouble() == sample.host_time_s);
    }
    assert(rows == 2);
  } else if (test == "capture-excursion") {
    p.qualify();
    p.publish(2);
    assert(!p.m_cancelled && p.isRunning());
    assert(p.releases == 0 && p.m_sensor_samples.size() == 1);
    assert(p.m_sensor_samples.back().torque_nm == 2);
    assert(p.m_band_status->text().contains("OUT OF BAND"));
    p.publish(0.1, 15);
    assert(p.m_sensor_samples.size() == 2 && p.isRunning());
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_acquisition_complete && p.m_export->isEnabled());
    assert(!p.m_power_label->text().contains("VALID"));
    QFile csv(p.m_sensor_path);
    assert(csv.open(QIODevice::ReadOnly));
    const QByteArray contents = csv.readAll();
    assert(!contents.contains("no_load_valid"));
    assert(contents.contains("# interrupted=0"));
  } else if (test == "rearm") {
    p.qualify();
    p.publish();
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.m_acquisition_complete && p.isRunning() && p.m_rearm->isEnabled());
    assert(p.zero_hold_commands == 0 && p.releases == 0);
    for (int n = 0; n < 10; ++n)
      p.publish();
    assert(p.triggers == 1);
    p.rearm();
    assert(!p.m_export->isEnabled());
    p.tick();
    assert(p.triggers == 1 && p.m_dwell_start < 0);
    p.publish();
    assert(p.triggers == 1);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish();
    assert(p.triggers == 1 && p.m_acquire->isEnabled());
    p.m_acquire->click();
    assert(p.triggers == 2);
  } else if (test == "supply-interval") {
    p.qualify();
    p.m_duration->setValue(0.5);
    p.m_last_fresh_wall = p.m_last_frame_wall = -1;
    std::this_thread::sleep_for(std::chrono::milliseconds(550));
    p.publish();
    assert(!p.m_supply_done && p.m_dataset_pending && p.isRunning());
    assert(p.m_sensor_samples.size() == 1);
    p.publish();
    assert(p.m_sensor_samples.size() == 2 && !p.m_supply_done);
  } else if (test == "speed-tolerance") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish(0.1, 15);
    assert(p.triggers == 0 && p.m_dwell_start < 0);
    p.frame.joints[0].velocity_deg_s = 120;
    p.publish();
    assert(p.triggers == 0 && p.m_dwell_start >= 0);
    assert(!p.m_band_status->text().contains("OUT OF BAND"));
  } else if (test == "speed-change-dwell") {
    assert(p.m_dwell_start >= 0 && !p.m_apply_rpm->isEnabled());
    p.m_live_rpm->setValue(20);
    assert(p.m_apply_rpm->isEnabled());
    p.expected_speed = 120;
    p.applySpeed();
    assert(p.m_target_rpm == 20 && p.m_dwell_start < 0);
    // Worker has not acknowledged yet: old target is still accepted.
    p.publish(0.1, 10);
    assert(p.isRunning() && p.m_speed_change_pending && p.m_dwell_start < 0);
    p.frame.joints[0].speed_hold_target_deg_s = 120;
    p.frame.joints[0].velocity_deg_s = 120;
    p.publish(0.1, 20);
    assert(p.isRunning() && !p.m_speed_change_pending);
    assert(p.m_dwell_start >= 0 && p.triggers == 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish(0.1, 20);
    assert(p.triggers == 0 && p.m_acquire->isEnabled());
    p.m_acquire->click();
    assert(p.triggers == 1 && p.releases == 0);
    p.buildMetadata();
    assert(p.m_metadata.contains("# target_rpm=20\n"));
  } else if (test == "speed-change-aborts-acquisition") {
    p.qualify();
    p.m_live_rpm->setValue(5);
    p.expected_speed = 30;
    p.applySpeed();
    p.frame.joints[0].speed_hold_target_deg_s = 30;
    p.frame.joints[0].velocity_deg_s = 30;
    p.publish(0.1, 5);
    assert(p.isRunning() && p.releases == 0);
    assert(!p.m_acquisition_complete && !p.m_dataset_pending &&
           p.m_phase == Probe::Phase::Complete);
    assert(p.m_result.contains("Speed changed"));
    assert(p.m_rearm->isEnabled() && p.triggers == 1);
    assert(p.zero_hold_commands == 0);
  } else if (test == "manual-acquire") {
    assert(p.m_acquire->isEnabled());
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish();
    assert(p.m_acquire->isEnabled() && p.triggers == 0);
    for (int n = 0; n < 10; ++n)
      p.publish();
    assert(p.triggers == 0 && p.m_acquire->isEnabled());
    // The operator can start outside the band, without waiting for a dwell.
    p.reader->m_latest = {p.reader->m_latest.t_s + 1, 2, 2, 10};
    p.startAcquisition();
    assert(p.triggers == 1 && !p.m_acquire->isEnabled());
    assert(p.m_band_status->text().contains("OUT OF BAND"));
    p.startAcquisition();
    assert(p.triggers == 1);
  } else if (test == "manual-acquire-before-dwell") {
    assert(p.m_acquire->isEnabled());
    assert(p.now() - p.m_dwell_start < p.m_dwell->value());
    p.m_acquire->click();
    assert(p.triggers == 1);
  } else if (test == "manual-acquire-stale") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish();
    assert(p.m_acquire->isEnabled());
    p.m_last_frame_wall = p.now() - 1;
    p.startAcquisition();
    assert(p.triggers == 0 && !p.isRunning());
  } else if (test == "soft-stop-dwell") {
    assert(p.m_soft_stop->text() == "Soft stop");
    assert(p.m_stop->text() == "Emergency stop");
    assert(p.m_stop->styleSheet().contains("#b71c1c"));
    assert(p.m_soft_stop->isEnabled());
    p.m_soft_stop->click();
    assert(p.isRunning() && p.releases == 0 && p.zero_hold_commands == 1);
    assert(p.m_target_rpm == 0 && p.m_dwell_start < 0);
    assert(!p.m_soft_stop->isEnabled() && p.m_stop->isEnabled());
    p.frame.joints[0].speed_hold_target_deg_s = 0;
    p.frame.joints[0].velocity_deg_s = 0;
    p.publish(0.1, 0);
    assert(p.triggers == 0 && !p.m_rearm->isEnabled());
    assert(p.m_phase == Probe::Phase::Complete && p.isRunning());
    p.softStop();
    assert(p.zero_hold_commands == 1);
    p.m_stop->click();
    assert(!p.isRunning() && p.releases == 1);
  } else if (test == "soft-stop-acquisition") {
    p.qualify();
    p.publish();
    p.m_soft_stop->click();
    assert(p.m_cancelled && !p.m_acquisition_complete);
    assert(p.isRunning() && p.zero_hold_commands == 1 && p.releases == 0);
    p.frame.joints[0].speed_hold_target_deg_s = 0;
    p.frame.joints[0].velocity_deg_s = 0;
    p.publish(0.1, 0);
    assert(!p.m_dataset_pending && !p.m_acquisition_complete && p.isRunning());
    assert(p.m_result.contains("Soft stop requested"));
    assert(QFile::exists(p.m_sensor_path));
    assert(!p.m_rearm->isEnabled() && p.m_export->isEnabled());
  } else if (test == "zero-hold-timeout") {
    p.qualify();
    p.publish();
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    p.m_live_rpm->setValue(0);
    p.m_apply_rpm->click();
    assert(p.m_target_rpm == 0 && p.zero_hold_commands == 1);
    p.m_speed_change_wall = p.now() - p.m_ramp->value() - 3;
    p.tick();
    assert(!p.isRunning() && p.releases == 1 && p.m_acquisition_complete);
    assert(p.m_status->text().contains("not acknowledged"));
  } else if (test == "manual-zero-after-completion") {
    p.qualify();
    p.publish();
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = true;
    p.finishAcquisition();
    assert(p.zero_hold_commands == 0 && p.m_target_rpm == 10);
    p.m_live_rpm->setValue(0);
    assert(p.m_apply_rpm->isEnabled());
    p.m_apply_rpm->click();
    assert(p.zero_hold_commands == 1 && p.releases == 0 && p.isRunning());
    p.frame.joints[0].speed_hold_target_deg_s = 0;
    p.frame.joints[0].velocity_deg_s = 0;
    p.publish(0.1, 0);
    assert(!p.m_speed_change_pending && !p.m_rearm->isEnabled());
    assert(p.m_acquisition_complete && p.m_export->isEnabled());
  } else if (test == "speed-change-mismatch") {
    p.m_live_rpm->setValue(20);
    p.expected_speed = 120;
    p.applySpeed();
    p.frame.joints[0].speed_hold_target_deg_s = 90;
    p.publish();
    assert(!p.isRunning() && p.releases == 1);
  } else {
    assert(false && "unknown test case");
  }
  std::cout << test << " PASS\n";
}
