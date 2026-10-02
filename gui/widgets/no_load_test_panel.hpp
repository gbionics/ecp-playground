// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
#pragma once

#include "actuator_test/external_daq.hpp"
#include "actuator_test/power_supply.hpp"
#include "core/controller_worker.hpp"

#include <QDialog>
#include <QElapsedTimer>
#include <deque>
#include <memory>
#include <vector>

QT_BEGIN_NAMESPACE
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QCheckBox;
class QLineEdit;
class QLabel;
class QPushButton;
class QGroupBox;
QT_END_NAMESPACE

namespace actuator_test::gui {

class StripChart;

class NoLoadTestDialog : public QDialog {
  Q_OBJECT
public:
  explicit NoLoadTestDialog(
      QWidget *parent = nullptr,
      QString result_root =
          QStringLiteral("/home/preddi/Documents/test-actuator-analysis"));
  void setJoints(const std::vector<JointInfo> &joints);
  void setExternalDaqReader(std::shared_ptr<ExternalDaqReader> reader);
  void appendTelemetry(const TelemetryFrame &frame);
  void setAvailable(bool available);
  bool isRunning() const { return m_running; }
  bool isBusy() const { return m_running || bool(m_supply); }
  void stopTest(const QString &reason = QStringLiteral("Stopped"));

signals:
  void speedHoldRequested(std::size_t joint, double velocity_deg_s,
                          double ramp_time_s, bool continuous_rotation,
                          bool release, int32_t pvt_kp, int32_t pvt_kd);
  void busyChanged();

protected:
  void closeEvent(QCloseEvent *event) override;
  void reject() override;

private:
  enum class Phase {
    Idle,
    WaitingMotion,
    Dwell,
    SupplyStarting,
    Acquiring,
    Complete
  };
  void start();
  void rearm();
  void tick();
  void updateControls();
  void loadJointGains();
  void buildMetadata();
  void plotTelemetry();
  void plotPower();
  void updateTorqueReadout(const ExternalDaqReader::Sample &sample,
                           double wall);
  void resetTorqueMean();
  // Overridable boundary permits regression tests without instrument commands.
  virtual void beginAcquisition();
  void finishAcquisition();
  void exportResults();
  bool saveCsv(const QString &path, bool supply);
  void persistResults();
  double now() const { return m_clock.elapsed() / 1000.0; }

  QGroupBox *m_settings;
  QComboBox *m_joint;
  QDoubleSpinBox *m_rpm, *m_ramp, *m_band, *m_dwell, *m_tolerance;
  QDoubleSpinBox *m_period, *m_duration;
  QSpinBox *m_kp, *m_kd;
  QCheckBox *m_continuous;
  QLineEdit *m_host;
  QLabel *m_status, *m_live, *m_saved, *m_power_label;
  StripChart *m_speed_chart, *m_torque_chart, *m_power_chart;
  int m_series_ref_speed = -1, m_series_drive_speed = -1;
  int m_series_daq_speed = -1, m_series_torque = -1;
  int m_series_band_hi = -1, m_series_band_lo = -1, m_series_power = -1;
  QPushButton *m_start, *m_stop, *m_rearm, *m_export;
  std::shared_ptr<ExternalDaqReader> m_daq;
  std::unique_ptr<PowerSupplyReader> m_supply;
  std::vector<ExternalDaqReader::Sample> m_sensor_samples;
  std::vector<PowerSupplyReader::Sample> m_supply_samples;
  std::vector<JointInfo> m_joints;
  TelemetryFrame m_frame;
  QElapsedTimer m_clock;
  Phase m_phase = Phase::Idle;
  std::size_t m_active_joint = 0;
  bool m_available = false, m_running = false, m_motion_ack = false;
  bool m_valid = false, m_cancelled = false, m_sensor_done = false;
  bool m_supply_done = false, m_supply_ok = false, m_interrupted = false;
  bool m_dataset_pending = false;
  double m_last_frame_wall = -1, m_last_daq_t = -1, m_last_fresh_wall = -1;
  double m_start_wall = 0, m_dwell_start = -1, m_capture_start = -1;
  double m_torque_mean_start = -1, m_torque_sum = 0;
  std::deque<double> m_torque_window;
  QString m_torque_mean_text;
  QString m_session, m_result, m_metadata, m_sensor_path, m_supply_path;
  QString m_result_root;
};
} // namespace actuator_test::gui
