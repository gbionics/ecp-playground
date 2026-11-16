// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Copper-loss / thermal-model characterisation: the operator manually applies
// a constant or cyclic current to one joint (same open-loop current command as the
// "Apply current" box in JogPanel) and the view records the motor/drive
// temperatures at a fixed, slow rate from the moment current is injected
// through cooling at 0 A until Release is pressed. The DC supply meter is
// polled continuously and plotted below the temperature on the same time
// axis. Trailing-window averages and temperature slopes flag thermal steady
// state; the operator records steady-state points manually.

#pragma once

#include "actuator_test/power_supply.hpp"
#include "actuator_test/external_daq.hpp"
#include "core/controller_worker.hpp"
#include "core/telemetry.hpp"

#include <QDialog>
#include <QString>
#include <memory>
#include <optional>
#include <vector>

QT_BEGIN_NAMESPACE
class QCheckBox;
class QCloseEvent;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QPushButton;
class QTableWidget;
class QTimer;
QT_END_NAMESPACE

namespace actuator_test::gui {

class StripChart;

class ThermalTestDialog : public QDialog {
  Q_OBJECT

public:
  explicit ThermalTestDialog(
      QWidget *parent = nullptr,
      const QString &save_directory =
          QStringLiteral("/home/preddi/Documents/test-actuator-analysis"));

  void setJoints(const std::vector<JointInfo> &joints);
  void appendTelemetry(const TelemetryFrame &frame);
  void setExternalDaqReader(std::shared_ptr<ExternalDaqReader> reader);

  /// True while recording heating or cooling.
  bool isRunning() const { return m_running; }
  /// Running, or the DC supply is being polled.
  bool isBusy() const;

signals:
  /// Same semantics as LockedRotorTestDialog::currentSetpointRequested.
  void currentSetpointRequested(std::size_t joint, double target_current_a,
                                bool release);
  /// Minimum-jerk transition from the present setpoint to `target_current_a`
  /// over `ramp_time_s`, then hold (same as LockedRotorTestDialog).
  void currentRampRequested(std::size_t joint, double target_current_a,
                            double ramp_time_s);
  void cyclicCurrentRequested(const CyclicCurrentCommand &command);

protected:
  void closeEvent(QCloseEvent *event) override;

private:
  struct Sample {
    double t_s = 0.0; ///< Seconds since current injection started.
    double commanded_current_a = 0.0;
    double actual_current_a = 0.0;
    int16_t motor_temp_c = -1;
    int16_t drive_temp_c = -1;
    bool steady = false;
    bool cyclic_active = false;
    double commanded_i2_a2s = 0.0;
    double actual_i2_a2s = 0.0;
    std::optional<TemperatureDaqReader::Sample> pt1000;
    std::optional<ExternalDaqReader::Sample> torsiometer;
    std::optional<ExternalDaqReader::TorqueMean> torque_mean;
    double torque_mean_window_s = 1.0;
  };

  struct ProfileRun {
    double requested_s = 0.0;
    std::optional<double> started_s;
    std::optional<double> ended_s;
    double kt_nm_a = 0.0;
    CyclicCurrentProfile profile;
  };

  struct PowerSample {
    double t_s = 0.0; ///< Same time base as Sample::t_s.
    double voltage_v = 0.0;
    double current_a = 0.0;
    double power_w = 0.0;
    double latency_s = 0.0;
  };

  /// Statistics over the trailing averaging window (which never reaches back
  /// before the last setpoint change).
  struct WindowStats {
    double t_end_s = 0.0;
    double window_start_s = 0.0;
    double span_s = 0.0; ///< Covered part of the window.
    bool window_full = false;
    int temp_samples = 0;
    double mean_cmd_a = 0.0;
    double mean_actual_a = 0.0;
    std::optional<double> mean_motor_c;
    std::optional<double> mean_drive_c;
    std::optional<double> slope_motor_c_min;
    std::optional<double> slope_drive_c_min;
    int power_samples = 0;
    double mean_v = 0.0;
    double mean_i = 0.0;
    double mean_p = 0.0;
    double std_p = 0.0;
    bool steady = false;
  };

  struct SteadyPoint {
    WindowStats stats;
    QString signal;
  };

  enum class SteadySignal { Motor = 0, Drive = 1, Both = 2 };

  void applyCurrent();
  void startCyclicProfile();
  void finishCyclicProfile();
  CyclicCurrentProfile configuredProfile() const;
  void refreshProfilePreview();
  void returnToZero();
  void release();
  void startRecording();
  void finishRecording(const QString &reason);
  void recordSample(double t_s, const JointTelemetry &jt);
  void updateLiveReadout(const JointTelemetry &jt);
  WindowStats computeStats() const;
  void refreshStats();
  void updateChartWindows();
  void updateRatedCurrent();
  void updateControls();
  std::size_t selectedJoint() const;
  double testTimeS() const;
  double windowS() const;

  void togglePowerSupply(bool connect);
  void pollPowerSupply();
  void recordSteadyPoint();
  void showWarning(const QString &title, const QString &message);
  void exportCsv();
  bool saveCsv(const QString &path);
  void autoSaveCsv();
  void toggleTemperatureDaq(bool enabled);
  void pollTemperatureDaq();
  std::optional<ExternalDaqReader::Sample> latestTorque() const;

  std::vector<JointInfo> m_joints;
  QString m_save_directory;
  QString m_kt_joint;

  QComboBox *m_joint_combo = nullptr;
  QLabel *m_rated_label = nullptr;
  QDoubleSpinBox *m_current_spin = nullptr;
  QDoubleSpinBox *m_rate_spin = nullptr;
  QDoubleSpinBox *m_ramp_spin = nullptr;
  QDoubleSpinBox *m_stance_spin = nullptr;
  QDoubleSpinBox *m_swing_spin = nullptr;
  QDoubleSpinBox *m_kt_spin = nullptr;
  QDoubleSpinBox *m_period_spin = nullptr;
  QDoubleSpinBox *m_duty_spin = nullptr;
  QDoubleSpinBox *m_transition_spin = nullptr;
  QDoubleSpinBox *m_duration_spin = nullptr;
  QPushButton *m_profile_btn = nullptr;
  QLabel *m_profile_label = nullptr;
  StripChart *m_profile_preview = nullptr;
  bool m_profile_requested = false;
  QCheckBox *m_confirm_check = nullptr;
  QPushButton *m_apply_btn = nullptr;
  QPushButton *m_zero_btn = nullptr;
  QPushButton *m_release_btn = nullptr;
  QPushButton *m_export_btn = nullptr;
  QLabel *m_status_label = nullptr;
  QLabel *m_live_label = nullptr;

  QComboBox *m_ss_signal_combo = nullptr;
  QDoubleSpinBox *m_ss_window_spin = nullptr;
  QDoubleSpinBox *m_ss_threshold_spin = nullptr;
  QLabel *m_ss_state_label = nullptr;
  QLabel *m_ss_detail_label = nullptr;
  QPushButton *m_mark_btn = nullptr;
  QTableWidget *m_points_table = nullptr;

  QLineEdit *m_psu_host_edit = nullptr;
  QDoubleSpinBox *m_psu_rate_spin = nullptr;
  QCheckBox *m_psu_auto_check = nullptr;
  QPushButton *m_psu_connect_btn = nullptr;
  bool m_psu_auto_started = false;
  QLabel *m_psu_status_label = nullptr;
  QLabel *m_psu_live_label = nullptr;
  QTimer *m_psu_timer = nullptr;
  std::unique_ptr<actuator_test::PowerSupplyPoller> m_poller;
  double m_psu_max_latency_s = 0.0;

  StripChart *m_temperature_chart = nullptr;
  QLineEdit *m_temperature_channel = nullptr;
  QPushButton *m_temperature_connect = nullptr;
  QLabel *m_temperature_live = nullptr;
  std::unique_ptr<TemperatureDaqReader> m_temperature_daq;
  std::shared_ptr<ExternalDaqReader> m_shared_daq;
  bool m_shared_temperature_enabled = false;
  std::optional<TemperatureDaqReader::Sample> m_pt1000;
  int m_series_pt1000 = -1;
  StripChart *m_current_chart = nullptr;
  StripChart *m_torque_chart = nullptr;
  QLabel *m_torque_live = nullptr;
  QDoubleSpinBox *m_torque_window = nullptr;
  int m_series_torque_mean = -1;
  int m_series_torque_raw = -1;
  int m_series_torque_filtered = -1;
  double m_last_torque_time_s = 0.0;
  int m_series_current = -1;
  std::optional<double> m_current_chart_last_s;
  StripChart *m_power_chart = nullptr;
  int m_series_motor = -1;
  int m_series_drive = -1;
  int m_series_power = -1;
  int m_series_power_mean = -1;

  bool m_running = false;
  bool m_returning_to_zero = false;
  bool m_control_observed = false;
  bool m_steady = false;
  std::size_t m_active_joint = 0;
  std::optional<double> m_t0_s;
  std::optional<double> m_t0_steady_s; ///< PowerSupplyPoller clock at m_t0_s.
  std::optional<double> m_next_sample_t_s;
  std::optional<double> m_zero_requested_t_s;
  /// Ramp time used by the pending "Return to 0 A" (extends the timeout).
  double m_zero_ramp_s = 0.0;
  std::optional<double> m_steady_since_s;
  double m_setpoint_t_s = 0.0; ///< Test time of the last setpoint change.
  double m_latest_frame_t_s = 0.0;
  double m_commanded_i2_origin = 0.0;
  double m_actual_i2_origin = 0.0;
  std::vector<Sample> m_samples;
  std::vector<PowerSample> m_power;
  std::vector<SteadyPoint> m_points;
  std::vector<ProfileRun> m_profiles;
};

} // namespace actuator_test::gui
