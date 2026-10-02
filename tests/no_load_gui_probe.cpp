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
#include <QPushButton>
#include <QSpinBox>
#include <QStandardPaths>
#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/controller_worker.hpp"
#include "widgets/plot_panel.hpp"

// Simulate reader publication without opening NI tasks, sockets or the bus.
#define private public
#include "widgets/no_load_test_panel.hpp"
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
              assert(speed == (release ? 0 : 60));
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
    tick();
  }

  void qualify() {
    assert(triggers == 0 && m_dwell_start >= 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    publish();
    assert(triggers == 1 && isRunning());
  }

private:
  void beginAcquisition() override {
    ++triggers;
    m_phase = Phase::Acquiring;
    m_capture_start = now();
    m_dataset_pending = true;
    m_cancelled = m_sensor_done = m_supply_done = m_supply_ok = false;
    m_valid = false;
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
  if (test == "torque-mean") {
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
    p.m_supply_done = p.m_supply_ok = p.m_sensor_done = true;
    p.finishAcquisition();
    assert(p.m_valid);
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
  } else if (test == "capture-excursion") {
    p.qualify();
    p.publish(2);
    assert(p.m_cancelled && !p.m_valid && !p.isRunning());
    assert(p.releases == 1 && !p.m_export->isEnabled());
  } else if (test == "rearm") {
    p.qualify();
    p.publish();
    p.m_supply_samples = {{0, 48, 1, 48}};
    p.m_supply_done = p.m_supply_ok = p.m_sensor_done = true;
    p.finishAcquisition();
    assert(p.m_valid && p.isRunning() && p.m_rearm->isEnabled());
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
    assert(p.triggers == 2);
  } else if (test == "supply-interval") {
    p.qualify();
    p.m_capture_start = p.now() - p.m_duration->value() - 1;
    p.publish();
    assert(!p.m_sensor_done && p.isRunning());
    assert(p.m_sensor_samples.size() == 1);
    p.publish();
    assert(p.m_sensor_samples.size() == 2 && !p.m_sensor_done);
  } else if (test == "speed-tolerance") {
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    p.publish(0.1, 15);
    assert(p.triggers == 0 && p.m_dwell_start < 0);
    p.frame.joints[0].velocity_deg_s = 120;
    p.publish();
    assert(p.triggers == 0 && p.m_dwell_start < 0);
  } else {
    assert(false && "unknown test case");
  }
  std::cout << test << " PASS\n";
}
