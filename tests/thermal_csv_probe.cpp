// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/thermal_test_panel.hpp"
#include "widgets/plot_panel.hpp"

#include <QApplication>
#include <QCheckBox>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QLabel>
#include <QMessageBox>
#include <QPushButton>
#include <QTemporaryDir>

#include <cassert>
#include <cmath>

using namespace actuator_test::gui;

static QPushButton *button(ThermalTestDialog &dialog, const QString &text) {
  for (auto *widget : dialog.findChildren<QPushButton *>()) {
    if (QString(widget->text()).remove('&') == text)
      return widget;
  }
  assert(false);
  return nullptr;
}

static QByteArray contents(const QString &path) {
  QFile file(path);
  assert(file.open(QIODevice::ReadOnly));
  return file.readAll();
}

static void prepare(ThermalTestDialog &dialog, bool apply = true) {
  JointInfo joint;
  joint.name = "test-joint";
  joint.selectable = true;
  joint.rated_current_a = 10.0;
  dialog.setJoints({joint});
  for (auto *widget : dialog.findChildren<QCheckBox *>()) {
    if (widget->text().contains("automatically"))
      widget->setChecked(false);
    if (widget->text().contains("I confirm"))
      widget->setChecked(true);
  }
  for (auto *widget : dialog.findChildren<QDoubleSpinBox *>()) {
    if (widget->accessibleName() == "Target current in amperes")
      widget->setValue(5.0);
  }
  if (apply) {
    button(dialog, "Apply current")->click();
    assert(dialog.isRunning());
  }
}

static void setSpin(ThermalTestDialog &dialog, const QString &name, double value) {
  for (auto *spin : dialog.findChildren<QDoubleSpinBox *>()) {
    if (spin->accessibleName() == name) {
      spin->setValue(value);
      return;
    }
  }
  assert(false);
}

int main(int argc, char **argv) {
  QApplication app(argc, argv);
  QTemporaryDir temp;
  assert(temp.isValid());
  QCoreApplication::setApplicationName("thermal-csv-probe");
  const QDir root(temp.path() + "/test-actuator-analysis");

  for (int mode = 0; mode < 5; ++mode) {
    ThermalTestDialog dialog(nullptr, root.absolutePath());
    prepare(dialog);
    TelemetryFrame frame;
    JointTelemetry joint;
    joint.name = "test-joint";
    joint.commanded_current_a = 5.0;
    joint.current_a = 4.9;
    joint.motor_temp_c = 30;
    joint.drive_temp_c = 31;
    joint.current_control_active = true;
    frame.joints = {joint};
    for (int i = 0; i < 6; ++i) {
      frame.t_s = i * 0.21;
      dialog.appendTelemetry(frame);
      auto *current_chart =
          dialog.findChild<QWidget *>("thermalCurrentChart");
      assert(current_chart);
      // StripChart has no Q_OBJECT, so find via QWidget before the typed cast.
      auto *chart = static_cast<StripChart *>(current_chart);
      assert(std::fabs(chart->dataMaxX() - frame.t_s) < 1e-9);
    }
    button(dialog, "Record steady-state point")->click();
    const QStringList before = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    if (mode == 0) {
      button(dialog, "Return to 0 A")->click();
      frame.t_s += 0.21;
      frame.joints[0].commanded_current_a = 0.0;
      frame.joints[0].current_transition_active = true;
      dialog.appendTelemetry(frame);
      assert(dialog.isRunning());
      frame.joints[0].current_transition_active = false;
      frame.joints[0].current_a = 0.0;
      frame.joints[0].motor_temp_c = 29;
      for (int i = 0; i < 10; ++i) {
        frame.t_s += 0.21;
        dialog.appendTelemetry(frame);
        assert(dialog.isRunning());
      }
      assert(root.entryList(QDir::Dirs | QDir::NoDotAndDotDot) == before);
      assert(!button(dialog, "Export CSV...")->isEnabled());
      button(dialog, "Apply current")->click();
      assert(dialog.isRunning());
      frame.t_s += 0.21;
      frame.joints[0].commanded_current_a = 5.0;
      dialog.appendTelemetry(frame);
      button(dialog, "Release")->click();
      frame.joints[0].commanded_current_a = 0.0;
    } else if (mode == 1) {
      frame.joints[0].fault = true;
    } else if (mode == 2) {
      button(dialog, "Release")->click();
    } else if (mode == 3) {
      dialog.show();
      dialog.close();
    } else {
      frame.joints[0].current_control_active = false;
    }
    frame.t_s += 0.21;
    dialog.appendTelemetry(frame);
    assert(!dialog.isRunning());
    const QStringList after = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    assert(after.size() == before.size() + 1);
    QString saved;
    for (const auto &name : after) {
      if (!before.contains(name))
        saved = root.filePath(name);
    }
    const auto csv = contents(saved + "/thermal-test.csv");
    assert(csv.startsWith("time_s,commanded_current_a,actual_current_a,"));
    assert(csv.contains("0,5,4.9,30,31,0,0,0,0,,,,,,,,1,,\n"));
    assert(csv.contains("torsiometer_raw_nm,torsiometer_filtered_nm,torsiometer_steady_time_s,"));
    assert(csv.contains("torsiometer_raw_mean_nm,torsiometer_mean_window_s,"
                        "torsiometer_mean_span_s,torsiometer_mean_samples\n"));
    const auto csv_lines = csv.trimmed().split('\n');
    assert(csv_lines[0].split(',').size() == 19);
    for (int i = 1; i < csv_lines.size(); ++i)
      assert(csv_lines[i].split(',').size() == 19);
    auto *torque_widget = dialog.findChild<QWidget *>("thermalTorqueChart");
    assert(torque_widget);
    assert(static_cast<StripChart *>(torque_widget)->dataMaxX() == 0.0);
    assert(csv.count('\n') == (mode == 0 ? 19 : mode == 2 || mode == 3 ? 7 : 8));
    if (mode == 0)
      assert(csv.contains(",0,0,29,31,"));
    const auto points = contents(saved + "/thermal-test-points.csv");
    assert(points.startsWith("point,time_s,window_start_s,"));
    assert(points.count('\n') == 2);
    assert(!QFile::exists(saved + "/thermal-test-supply.csv"));
  }

  for (int mode = 0; mode < 5; ++mode) {
    ThermalTestDialog dialog(nullptr, root.absolutePath());
    prepare(dialog, false);
    CyclicCurrentCommand command;
    int sent = 0;
    int ramp_requests = 0;
    double ramp_target = 0.0;
    QObject::connect(&dialog, &ThermalTestDialog::cyclicCurrentRequested,
                     [&](const CyclicCurrentCommand &c) {
                       command = c;
                       ++sent;
                     });
    QObject::connect(&dialog, &ThermalTestDialog::currentRampRequested,
                     [&](std::size_t, double target, double) {
                       ++ramp_requests;
                       ramp_target = target;
                     });
    setSpin(dialog, "Output Kt:", 2.0);
    JointInfo refreshed;
    refreshed.name = "test-joint";
    refreshed.selectable = true;
    refreshed.rated_current_a = 10.0;
    dialog.setJoints({refreshed}); // Periodic joint info must not erase edited Kt.
    setSpin(dialog, "Stance torque:", 8.0);
    setSpin(dialog, "Swing torque:", -4.0);
    setSpin(dialog, "Cycling duration:", 1.3);
    // Invalid phase timing and excessive currents must not start recording.
    setSpin(dialog, "Each transition:", 1.0);
    button(dialog, "Start cyclic profile")->click();
    assert(sent == 0 && !dialog.isRunning());
    setSpin(dialog, "Each transition:", 0.1);
    setSpin(dialog, "Stance torque:", 22.0);
    button(dialog, "Start cyclic profile")->click();
    assert(sent == 0 && !dialog.isRunning());
    setSpin(dialog, "Stance torque:", 8.0);
    const auto before = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    button(dialog, "Start cyclic profile")->click();
    assert(sent == 1 && dialog.isRunning());
    assert(command.joint == 0 && command.profile.stance_a == 4.0);
    assert(command.profile.swing_a == -2.0);
    assert(!button(dialog, "Start cyclic profile")->isEnabled());
    double integral = 0.0;
    for (int i = 0; i < 401 && dialog.isRunning(); ++i) {
      const double t = i * 0.01;
      TelemetryFrame frame;
      frame.t_s = t;
      frame.joints.resize(1);
      auto &jt = frame.joints[0];
      jt.current_control_active = true;
      jt.cyclic_current_active = mode != 3 && t < command.profile.totalDurationS();
      jt.commanded_current_a = command.profile.value(t, 0.0);
      jt.current_a = jt.commanded_current_a;
      jt.motor_temp_c = 30;
      jt.drive_temp_c = 31;
      integral += jt.current_a * jt.current_a * 0.01;
      jt.commanded_i2_a2s = integral;
      jt.actual_i2_a2s = integral;
      if (mode == 1 && i >= 60) {
        jt.cyclic_current_active = false;
        jt.commanded_current_a = jt.current_a = 0.0;
      }
      if (mode == 4 && i >= 60) {
        jt.cyclic_current_active = false;
        jt.commanded_current_a = jt.current_a = 5.0;
      }
      dialog.appendTelemetry(frame);
      if (i == 60) {
        auto *widget = dialog.findChild<QWidget *>("thermalCurrentChart");
        assert(widget);
        assert(std::fabs(static_cast<StripChart *>(widget)->dataMaxX() - t) < 1e-9);
        if (mode == 1)
          button(dialog, "Return to 0 A")->click();
        if (mode == 2)
          button(dialog, "Release")->click();
        if (mode == 4)
          button(dialog, "Apply current")->click();
      }
    }
    if (mode == 0 || mode == 1 || mode == 4) {
      assert(dialog.isRunning()); // Includes cooling after natural/manual end.
      button(dialog, "Release")->click();
    }
    if (mode == 1 || mode == 4) {
      assert(ramp_requests == 1);
      assert(ramp_target == (mode == 1 ? 0.0 : 5.0));
    }
    assert(!dialog.isRunning());
    const auto after = root.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    assert(after.size() == before.size() + 1);
    QString saved;
    for (const auto &name : after)
      if (!before.contains(name))
        saved = root.filePath(name);
    const auto csv = contents(saved + "/thermal-test.csv");
    assert(csv.contains("cyclic_active,commanded_i2_a2s,actual_i2_a2s,pt1000_voltage_v,"));
    const auto rows = csv.trimmed().split('\n');
    if (mode == 0) {
      const auto last = rows.back().split(',');
      assert(last[6] == "0");
      assert(std::fabs(last[7].toDouble() - integral) < 0.001);
      assert(std::fabs(last[8].toDouble() - integral) < 0.001);
    }
    const auto metadata = contents(saved + "/thermal-test-profiles.csv");
    assert(metadata.count('\n') == 2);
    const auto fields = metadata.trimmed().split('\n').back().split(',');
    assert(fields[3].toDouble() == 2.0);
    assert(fields[4].toDouble() == 4.0);
    assert(fields[5].toDouble() == -2.0);
    assert(!fields[2].isEmpty());
    assert(fields[1].isEmpty() == (mode == 3));
  }

  QFile blocker(temp.path() + "/not-a-directory");
  assert(blocker.open(QIODevice::WriteOnly));
  blocker.close();
  ThermalTestDialog dialog(nullptr, blocker.fileName());
  prepare(dialog);
  TelemetryFrame frame;
  frame.joints.resize(1);
  frame.joints[0].commanded_current_a = 5.0;
  frame.joints[0].current_control_active = true;
  dialog.appendTelemetry(frame);
  button(dialog, "Release")->click();
  assert(button(dialog, "Export CSV...")->isEnabled());
  assert(!dialog.findChildren<QMessageBox *>().empty());
  bool failed = false;
  for (auto *label : dialog.findChildren<QLabel *>())
    failed |= label->text().contains("Automatic CSV save failed");
  assert(failed);
}
