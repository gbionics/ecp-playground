// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Top-level window: hosts the dockable control panels, the live plots and a
// telemetry table, and bridges panel signals to ControllerWorker commands via
// a periodic GUI-thread poll of the worker's snapshot/event channels.
//
// Enhanced with: adaptive update scheduling, professional theming, velocity/
// acceleration plots, CiA-402 state visualization, diagnostics panels, and
// comprehensive event logging.

#pragma once

#include "core/controller_worker.hpp"
#include "core/update_scheduler.hpp"

#include "actuator_test/settings.hpp"

#include <QMainWindow>
#include <QMap>
#include <QStringList>
#include <memory>

QT_BEGIN_NAMESPACE
class QPlainTextEdit;
class QTableWidget;
class QLabel;
class QPushButton;
class QTimer;
class QShortcut;
QT_END_NAMESPACE

namespace actuator_test::gui
{

class ConnectionPanel;
class JogPanel;
class LimitsPanel;
class TrajectoryPanel;
class PlotPanel;
class DrivesDiagnosticsPanel;
class EnhancedLimitsPanel;
class EventLogPanel;
class AxisOverviewPanel;

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(RuntimeProfile profile, QString default_config, QWidget *parent = nullptr);
    ~MainWindow() override;

    /// Post a connect command (used by the onboarding wizard).
    void requestConnect(const QString &config_path);

private slots:
    void poll();

private:
    struct JointRunStats
    {
        int samples = 0;
        double sum_err_sq = 0.0;
        double peak_abs_err = 0.0;
        double peak_abs_vel = 0.0;
        double min_seen_deg = 0.0;
        double max_seen_deg = 0.0;
        bool init = false;
    };

    struct SessionPreset
    {
        QString config_path;
        QStringList selected_joint_names;
        int trajectory_mode = 0;
        bool logging_enabled = true;
    };

    void buildDocks();
    void buildMenu();
    void wireSignals();
    void initializeUpdateScheduler();
    void refreshJoints(const std::vector<JointInfo> &joints);
    void applyState(ControllerState state);
    void appendLog(const QString &line);
    void updateTelemetry(const TelemetryFrame &frame);
    void checkLimitViolations(const std::vector<JointTelemetry> &joints);
    SessionPreset collectPreset() const;
    void applyPreset(const SessionPreset &preset);
    void savePresetToFile();
    void loadPresetFromFile();
    void exportOffsetsXml();
    void exportOffsetsXmlToPath(const QString &path, bool silent = false);
    void updateProfiler(const TelemetryFrame &frame);

    // --- Core components ---
    std::unique_ptr<ControllerWorker> m_worker;
    std::unique_ptr<UpdateScheduler> m_scheduler;
    RuntimeProfile m_profile;
    QString m_default_config;

    // --- Original panels ---
    ConnectionPanel *m_connection = nullptr;
    JogPanel *m_jog = nullptr;
    LimitsPanel *m_limits = nullptr;
    TrajectoryPanel *m_trajectory = nullptr;
    PlotPanel *m_plot = nullptr;

    // --- New enhanced/professional panels ---
    AxisOverviewPanel *m_axis_overview = nullptr;
    DrivesDiagnosticsPanel *m_drives_diagnostics = nullptr;
    EnhancedLimitsPanel *m_enhanced_limits = nullptr;
    EventLogPanel *m_event_log = nullptr;

    // --- Display tables and logs ---
    QTableWidget *m_table = nullptr;
    QTableWidget *m_profiler_table = nullptr;
    QPlainTextEdit *m_log = nullptr;

    // --- Status bar widgets ---
    QLabel *m_state_label = nullptr;
    QLabel *m_metrics_label = nullptr;
    QLabel *m_health_indicator = nullptr;
    QPushButton *m_estop_btn = nullptr;
    QPushButton *m_store_homing_btn = nullptr;

    // --- Timer and shortcuts ---
    QTimer *m_timer = nullptr;
    QShortcut *m_shortcut_estop = nullptr;
    QShortcut *m_shortcut_stop = nullptr;
    QShortcut *m_shortcut_pause = nullptr;
    QShortcut *m_shortcut_play = nullptr;

    // --- State tracking ---
    QStringList m_joint_names;
    QMap<QString, JointRunStats> m_joint_stats;
    ControllerState m_last_state = ControllerState::Disconnected;
    std::vector<JointTelemetry> m_last_telemetry;
    uint32_t m_update_scheduler_id_plot = 0;
    uint32_t m_update_scheduler_id_diagnostics = 0;
    uint32_t m_update_scheduler_id_telemetry = 0;
};

} // namespace actuator_test::gui
