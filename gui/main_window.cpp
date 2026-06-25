// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "main_window.hpp"

#include "widgets/connection_panel.hpp"
#include "widgets/jog_panel.hpp"
#include "widgets/limits_panel.hpp"
#include "widgets/plot_panel.hpp"
#include "widgets/trajectory_panel.hpp"
#include "widgets/axis_overview_panel.hpp"
#include "widgets/drives_diagnostics_panel.hpp"
#include "widgets/enhanced_limits_panel.hpp"
#include "widgets/event_log_panel.hpp"

#include <QDockWidget>
#include <QDir>
#include <QFileDialog>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeySequence>
#include <QLabel>
#include <QMap>
#include <QMessageBox>
#include <QMenuBar>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QShortcut>
#include <QStatusBar>
#include <QTableWidget>
#include <QTimer>
#include <QFile>
#include <QTextStream>
#include <QApplication>
#include <chrono>

#include <cmath>

namespace actuator_test::gui
{

namespace
{

QString tempString(int16_t t)
{
    return (t < 0) ? QStringLiteral("--") : QString::number(t) + QStringLiteral("C");
}

QString tupleQuoted(const QStringList &values)
{
    QStringList quoted;
    quoted.reserve(values.size());
    for (const auto &v : values)
    {
        quoted << QStringLiteral("\"") + v + QStringLiteral("\"");
    }
    return QStringLiteral("(") + quoted.join(QStringLiteral(" ")) + QStringLiteral(")");
}

QString tupleNumbers(const QList<double> &values, int decimals)
{
    QStringList out;
    out.reserve(values.size());
    for (double v : values)
    {
        out << QString::number(v, 'f', decimals);
    }
    return QStringLiteral("(") + out.join(QStringLiteral(" ")) + QStringLiteral(")");
}

} // namespace

MainWindow::MainWindow(RuntimeProfile profile, QString default_config, QWidget *parent)
    : QMainWindow(parent), m_profile(profile), m_default_config(std::move(default_config))
{
    setWindowTitle(tr("Actuator Test - Motion Control Platform"));
    resize(1440, 900);

    // Initialize update scheduler
    m_scheduler = std::make_unique<UpdateScheduler>();

    m_worker = std::make_unique<ControllerWorker>(m_profile);
    m_worker->start();

    m_plot = new PlotPanel();
    m_plot->setUpdateRate(50); // Update every 50ms instead of 16ms for reduced CPU
    setCentralWidget(m_plot);

    buildMenu();
    buildDocks();
    initializeUpdateScheduler();
    wireSignals();

    // Enhanced status bar
    m_state_label = new QLabel(tr("disconnected"));
    statusBar()->addPermanentWidget(m_state_label);

    m_health_indicator = new QLabel(tr("●"));
    m_health_indicator->setStyleSheet("color: red; font-size: 12px; font-weight: bold;");
    m_health_indicator->setToolTip(tr("System health status"));
    statusBar()->addPermanentWidget(m_health_indicator);

    m_metrics_label = new QLabel(tr("t=0.00s | selected=0"));
    statusBar()->addPermanentWidget(m_metrics_label);

    m_estop_btn = new QPushButton(tr("EMERGENCY STOP"));
    m_estop_btn->setMinimumHeight(36);
    m_estop_btn->setMinimumWidth(220);
    m_estop_btn->setStyleSheet(
        "QPushButton { background:#c1121f; color:white; font-weight:700; border:2px solid #7f1d1d; "
        "border-radius:6px; padding:6px 12px; }"
        "QPushButton:pressed { background:#9b0d18; }"
        "QPushButton:disabled { background:#6b7280; border-color:#4b5563; color:#e5e7eb; }");
    m_estop_btn->setToolTip(tr("Immediately stop all motion and idle all drives. Shortcut: Ctrl+Shift+E"));
    statusBar()->addPermanentWidget(m_estop_btn);

    m_store_homing_btn = new QPushButton(tr("Store Homing"));
    m_store_homing_btn->setMinimumHeight(30);
    m_store_homing_btn->setToolTip(tr("Save current offsets/limits to build/joint-offsets.xml"));
    statusBar()->addPermanentWidget(m_store_homing_btn);

    m_shortcut_estop = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Shift+E")), this);
    m_shortcut_stop = new QShortcut(QKeySequence(QStringLiteral("Esc")), this);
    m_shortcut_pause = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Space")), this);
    m_shortcut_play = new QShortcut(QKeySequence(QStringLiteral("Ctrl+Return")), this);

    m_timer = new QTimer(this);
    m_timer->setInterval(16); // Main poll loop - still 60 Hz but delegates to scheduler
    connect(m_timer, &QTimer::timeout, this, &MainWindow::poll);
    m_timer->start();
}

MainWindow::~MainWindow() = default;

void MainWindow::buildMenu()
{
    auto *file_menu = menuBar()->addMenu(tr("File"));
    auto *save_preset = file_menu->addAction(tr("Save Session Preset..."));
    auto *load_preset = file_menu->addAction(tr("Load Session Preset..."));
    file_menu->addSeparator();
    auto *export_xml = file_menu->addAction(tr("Export Joint Offsets XML..."));
    auto *store_homing = file_menu->addAction(tr("Store Homing Snapshot"));

    connect(save_preset, &QAction::triggered, this, &MainWindow::savePresetToFile);
    connect(load_preset, &QAction::triggered, this, &MainWindow::loadPresetFromFile);
    connect(export_xml, &QAction::triggered, this, &MainWindow::exportOffsetsXml);
    connect(store_homing, &QAction::triggered, this, [this] {
        exportOffsetsXmlToPath(QDir::currentPath() + QStringLiteral("/build/joint-offsets.xml"), false);
    });
}

void MainWindow::buildDocks()
{
    auto addDock = [this](const QString &title, QWidget *w, Qt::DockWidgetArea area, bool tabify = false) {
        auto *dock = new QDockWidget(title, this);
        dock->setWidget(w);
        addDockWidget(area, dock);
        return dock;
    };

    // === LEFT PANEL: Control and Configuration ===
    m_connection = new ConnectionPanel(m_default_config);
    m_jog = new JogPanel();
    m_limits = new LimitsPanel();
    m_enhanced_limits = new EnhancedLimitsPanel();
    m_trajectory = new TrajectoryPanel();

    addDock(tr("Connection"), m_connection, Qt::LeftDockWidgetArea);
    auto *jog_dock = addDock(tr("Jog / Home"), m_jog, Qt::LeftDockWidgetArea);
    auto *lim_dock = addDock(tr("Limits (Legacy)"), m_limits, Qt::LeftDockWidgetArea);
    auto *enh_lim_dock = addDock(tr("Limits (Enhanced)"), m_enhanced_limits, Qt::LeftDockWidgetArea);
    auto *traj_dock = addDock(tr("Trajectory"), m_trajectory, Qt::LeftDockWidgetArea);
    
    tabifyDockWidget(jog_dock, lim_dock);
    tabifyDockWidget(lim_dock, enh_lim_dock);
    tabifyDockWidget(enh_lim_dock, traj_dock);
    jog_dock->raise();

    // === RIGHT PANEL: Diagnostics and Overview ===
    m_axis_overview = new AxisOverviewPanel();
    m_drives_diagnostics = new DrivesDiagnosticsPanel();

    auto *overview_dock = addDock(tr("Axis Overview"), m_axis_overview, Qt::RightDockWidgetArea);
    auto *diag_dock = addDock(tr("Drive Diagnostics"), m_drives_diagnostics, Qt::RightDockWidgetArea);
    tabifyDockWidget(overview_dock, diag_dock);
    overview_dock->raise();

    // === BOTTOM PANEL: Monitoring and Logs ===
    m_table = new QTableWidget(0, 10);
    m_table->setHorizontalHeaderLabels({tr("Joint"), tr("Ref [deg]"), tr("Actual [deg]"), tr("Error [deg]"),
                                        tr("Vel [deg/s]"), tr("Min [deg]"), tr("Max [deg]"), tr("Motor"),
                                        tr("Drive"), tr("Err")});
    m_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    addDock(tr("Telemetry"), m_table, Qt::BottomDockWidgetArea);

    m_profiler_table = new QTableWidget(0, 6);
    m_profiler_table->setHorizontalHeaderLabels({tr("Joint"), tr("RMS Err [deg]"), tr("Peak Err [deg]"),
                                                 tr("Peak Vel [deg/s]"), tr("Range Used [deg]"), tr("Samples")});
    m_profiler_table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    m_profiler_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
    auto *profiler_dock = addDock(tr("Profiler"), m_profiler_table, Qt::BottomDockWidgetArea);

    m_log = new QPlainTextEdit();
    m_log->setReadOnly(true);
    m_log->setMaximumBlockCount(2000);
    auto *log_dock = addDock(tr("Log"), m_log, Qt::BottomDockWidgetArea);

    m_event_log = new EventLogPanel();
    auto *event_dock = addDock(tr("Event Log"), m_event_log, Qt::BottomDockWidgetArea);

    tabifyDockWidget(profiler_dock, log_dock);
    tabifyDockWidget(log_dock, event_dock);
    profiler_dock->raise();
}

void MainWindow::initializeUpdateScheduler()
{
    // Register panels with configurable update rates
    // Plot panel: 50ms (20 Hz) - smooth visualization without excessive repaints
    m_update_scheduler_id_plot = m_scheduler->registerPanel(
        "PlotPanel", 50, [this]() {
            // Handled by plot panel's internal update rate
        });

    // Diagnostics: 100ms (10 Hz) - less frequent updates for diagnostics
    m_update_scheduler_id_diagnostics = m_scheduler->registerPanel(
        "Diagnostics", 100, [this]() {
            if (!m_last_telemetry.empty())
            {
                m_drives_diagnostics->updateTelemetry(m_last_telemetry);
                m_axis_overview->updateTelemetry(m_last_telemetry);
            }
        });

    // Telemetry table: 100ms (10 Hz) - information updates
    m_update_scheduler_id_telemetry = m_scheduler->registerPanel(
        "Telemetry", 100, [this]() {
            // Table update happens in updateTelemetry
        });
}

void MainWindow::wireSignals()
{
    connect(m_connection, &ConnectionPanel::connectRequested, this,
            [this](const QString &p) { m_worker->post(ConnectCommand{p.toStdString()}); });
    connect(m_connection, &ConnectionPanel::disconnectRequested, this,
            [this] { m_worker->post(DisconnectCommand{}); });

    connect(m_jog, &JogPanel::jogRequested, this,
            [this](std::size_t j, double v) { m_worker->post(JogCommand{j, v}); });
    connect(m_jog, &JogPanel::goToRequested, this,
            [this](std::size_t j, double d, double s) { m_worker->post(GoToCommand{j, d, s}); });
    connect(m_jog, &JogPanel::stopRequested, this, [this] { m_worker->post(StopCommand{}); });

    connect(m_limits, &LimitsPanel::captureToggled, this, [this](bool start) {
        m_worker->post(CaptureLimitsCommand{m_connection->selectedIndices(), start});
    });
    connect(m_limits, &LimitsPanel::setLimitsRequested, this,
            [this](std::size_t j, double mn, double mx) { m_worker->post(SetLimitsCommand{j, mn, mx}); });
    connect(m_limits, &LimitsPanel::resetLimitsRequested, this,
            [this](std::size_t j) { m_worker->post(ResetLimitsCommand{j}); });

    // Enhanced limits panel signals
    connect(m_enhanced_limits, &EnhancedLimitsPanel::captureToggled, this, [this](bool start) {
        m_worker->post(CaptureLimitsCommand{{m_enhanced_limits->currentJoint()}, start});
        });
        connect(m_enhanced_limits, &EnhancedLimitsPanel::setLimitsRequested, this,
            [this](std::size_t j, double mn, double mx) { m_worker->post(SetLimitsCommand{j, mn, mx}); });
        connect(m_enhanced_limits, &EnhancedLimitsPanel::resetLimitsRequested, this,
            [this](std::size_t j) { m_worker->post(ResetLimitsCommand{j}); });

    connect(m_trajectory, &TrajectoryPanel::playRequested, this, [this](TrajectoryMode mode, bool log) {
        m_worker->post(StartTrajectoryCommand{m_connection->selectedIndices(), mode, log});
    });
    connect(m_trajectory, &TrajectoryPanel::splineRecordToggled, this,
            [this](bool start) { m_worker->post(CaptureLimitsCommand{m_connection->selectedIndices(), start}); });
    connect(m_trajectory, &TrajectoryPanel::pauseRequested, this, [this] { m_worker->post(PauseCommand{}); });
    connect(m_trajectory, &TrajectoryPanel::stopRequested, this, [this] { m_worker->post(StopCommand{}); });

    connect(m_estop_btn, &QPushButton::clicked, this, [this] { m_worker->post(StopCommand{}); });
    connect(m_store_homing_btn, &QPushButton::clicked, this, [this] {
        exportOffsetsXmlToPath(QDir::currentPath() + QStringLiteral("/build/joint-offsets.xml"), false);
    });

    connect(m_shortcut_estop, &QShortcut::activated, this, [this] {
        if (m_estop_btn->isEnabled())
        {
            m_worker->post(StopCommand{});
        }
    });
    connect(m_shortcut_stop, &QShortcut::activated, this, [this] {
        if (m_last_state != ControllerState::Disconnected)
        {
            m_worker->post(StopCommand{});
        }
    });
    connect(m_shortcut_pause, &QShortcut::activated, this, [this] {
        if (m_last_state == ControllerState::Running)
        {
            m_worker->post(PauseCommand{});
        }
    });
    connect(m_shortcut_play, &QShortcut::activated, this, [this] {
        if (m_last_state != ControllerState::Disconnected)
        {
            m_worker->post(StartTrajectoryCommand{m_connection->selectedIndices(), m_trajectory->selectedMode(),
                                                  m_trajectory->loggingEnabled()});
        }
    });
}

void MainWindow::requestConnect(const QString &config_path)
{
    m_worker->post(ConnectCommand{config_path.toStdString()});
}

void MainWindow::poll()
{
    for (const auto &ev : m_worker->drainEvents())
    {
        switch (ev.kind)
        {
        case WorkerEvent::Kind::Log:
            appendLog(QString::fromStdString(ev.message));
            break;
        case WorkerEvent::Kind::Error:
            appendLog(QStringLiteral("[error] ") + QString::fromStdString(ev.message));
            break;
        case WorkerEvent::Kind::StateChanged:
            applyState(ev.state);
            break;
        }
    }

    const TelemetryFrame frame = m_worker->snapshot();
    m_last_telemetry = frame.joints;

    // Refresh joint lists when the enumerated set changes.
    const std::vector<JointInfo> joints = m_worker->joints();
    QStringList names;
    names.reserve(static_cast<int>(joints.size()));
    for (const auto &j : joints)
    {
        names << QString::fromStdString(j.name);
    }
    if (names != m_joint_names)
    {
        m_joint_names = names;
        refreshJoints(joints);
    }

    updateTelemetry(frame);
    updateProfiler(frame);
    m_jog->updateLiveLimits(joints);
    m_limits->updateLiveLimits(joints);
    m_enhanced_limits->updateLiveLimits(joints, frame.joints);
    m_plot->appendFrame(frame);
    m_scheduler->poll(static_cast<uint32_t>(m_timer->interval()));

    m_metrics_label->setText(
        tr("t=%1s | joints=%2 | selected=%3")
            .arg(QString::number(frame.t_s, 'f', 2))
            .arg(joints.size())
            .arg(m_connection->selectedIndices().size()));
}

void MainWindow::refreshJoints(const std::vector<JointInfo> &joints)
{
    m_connection->setJoints(joints);
    m_jog->setJoints(joints);
    m_limits->setJoints(joints);
    m_enhanced_limits->setJoints(joints);
    m_axis_overview->setJoints(joints);
    m_drives_diagnostics->setJoints(joints);
    m_plot->setJoints(m_joint_names);

    m_table->setRowCount(static_cast<int>(joints.size()));
    for (int i = 0; i < static_cast<int>(joints.size()); ++i)
    {
        m_table->setItem(i, 0, new QTableWidgetItem(QString::fromStdString(joints[static_cast<std::size_t>(i)].name)));
    }
}

void MainWindow::applyState(ControllerState state)
{
    m_last_state = state;
    m_state_label->setText(QString::fromUtf8(to_string(state)));

    m_connection->setState(state);
    m_limits->setState(state);
    m_enhanced_limits->setState(state);

    const bool connected = (state != ControllerState::Disconnected);
    const bool busy = (state == ControllerState::Running);
    m_jog->setEnabledControls(connected && !busy);
    m_trajectory->setEnabled(connected);
    m_trajectory->setRunning(state == ControllerState::Running);
    m_trajectory->setCapturing(state == ControllerState::Capturing);
    m_estop_btn->setEnabled(connected);
    m_store_homing_btn->setEnabled(connected);
}

void MainWindow::appendLog(const QString &line)
{
    m_log->appendPlainText(line);
}

void MainWindow::updateTelemetry(const TelemetryFrame &frame)
{
    if (static_cast<int>(frame.joints.size()) != m_table->rowCount())
    {
        return; // Table not yet in sync with the latest joint set.
    }
    for (int i = 0; i < static_cast<int>(frame.joints.size()); ++i)
    {
        const JointTelemetry &j = frame.joints[static_cast<std::size_t>(i)];
        auto setCell = [this, i](int col, const QString &text) {
            if (auto *item = m_table->item(i, col))
            {
                item->setText(text);
            }
            else
            {
                m_table->setItem(i, col, new QTableWidgetItem(text));
            }
        };
        setCell(0, QString::fromStdString(j.name));
        setCell(1, QString::number(j.reference_deg, 'f', 2));
        setCell(2, QString::number(j.actual_deg, 'f', 2));
        setCell(3, QString::number(j.error_deg, 'f', 3));
        setCell(4, QString::number(j.velocity_deg_s, 'f', 1));
        setCell(5, QString::number(j.min_limit_deg, 'f', 2));
        setCell(6, QString::number(j.max_limit_deg, 'f', 2));
        setCell(7, tempString(j.motor_temp_c));
        setCell(8, tempString(j.drive_temp_c));
        setCell(9, j.fault ? QStringLiteral("FAULT") : QString::number(j.error_code));
    }
}

void MainWindow::updateProfiler(const TelemetryFrame &frame)
{
    for (const auto &jt : frame.joints)
    {
        const QString name = QString::fromStdString(jt.name);
        auto &st = m_joint_stats[name];
        st.samples += 1;
        st.sum_err_sq += jt.error_deg * jt.error_deg;
        st.peak_abs_err = std::max(st.peak_abs_err, std::fabs(jt.error_deg));
        st.peak_abs_vel = std::max(st.peak_abs_vel, std::fabs(jt.velocity_deg_s));
        if (!st.init)
        {
            st.min_seen_deg = jt.actual_deg;
            st.max_seen_deg = jt.actual_deg;
            st.init = true;
        }
        st.min_seen_deg = std::min(st.min_seen_deg, jt.actual_deg);
        st.max_seen_deg = std::max(st.max_seen_deg, jt.actual_deg);
    }

    m_profiler_table->setRowCount(static_cast<int>(frame.joints.size()));
    for (int i = 0; i < static_cast<int>(frame.joints.size()); ++i)
    {
        const auto &jt = frame.joints[static_cast<std::size_t>(i)];
        const QString name = QString::fromStdString(jt.name);
        const auto it = m_joint_stats.constFind(name);
        if (it == m_joint_stats.cend())
        {
            continue;
        }
        const JointRunStats &st = it.value();
        const double rms = (st.samples > 0) ? std::sqrt(st.sum_err_sq / static_cast<double>(st.samples)) : 0.0;
        const double used = st.max_seen_deg - st.min_seen_deg;

        auto setCell = [this, i](int col, const QString &text) {
            if (auto *item = m_profiler_table->item(i, col))
            {
                item->setText(text);
            }
            else
            {
                m_profiler_table->setItem(i, col, new QTableWidgetItem(text));
            }
        };
        setCell(0, name);
        setCell(1, QString::number(rms, 'f', 3));
        setCell(2, QString::number(st.peak_abs_err, 'f', 3));
        setCell(3, QString::number(st.peak_abs_vel, 'f', 2));
        setCell(4, QString::number(used, 'f', 2));
        setCell(5, QString::number(st.samples));
    }
}

MainWindow::SessionPreset MainWindow::collectPreset() const
{
    SessionPreset preset;
    preset.config_path = m_connection->configPath();
    preset.selected_joint_names = m_connection->selectedJointNames();
    preset.trajectory_mode = static_cast<int>(m_trajectory->selectedMode());
    preset.logging_enabled = m_trajectory->loggingEnabled();
    return preset;
}

void MainWindow::applyPreset(const SessionPreset &preset)
{
    m_connection->setConfigPath(preset.config_path);
    m_trajectory->setSelectedMode(static_cast<TrajectoryMode>(preset.trajectory_mode));
    m_trajectory->setLoggingEnabled(preset.logging_enabled);
    m_connection->setSelectedJointNames(preset.selected_joint_names);
}

void MainWindow::savePresetToFile()
{
    const QString path = QFileDialog::getSaveFileName(this, tr("Save session preset"),
                                                      QStringLiteral("session-preset.json"),
                                                      tr("Preset JSON (*.json)"));
    if (path.isEmpty())
    {
        return;
    }

    const SessionPreset preset = collectPreset();
    QJsonObject root;
    root.insert(QStringLiteral("config_path"), preset.config_path);
    QJsonArray names;
    for (const auto &n : preset.selected_joint_names)
    {
        names.push_back(n);
    }
    root.insert(QStringLiteral("selected_joint_names"), names);
    root.insert(QStringLiteral("trajectory_mode"), preset.trajectory_mode);
    root.insert(QStringLiteral("logging_enabled"), preset.logging_enabled);

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
    {
        QMessageBox::warning(this, tr("Preset save failed"), tr("Cannot write preset file."));
        return;
    }
    f.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    f.close();
    appendLog(QStringLiteral("saved preset -> ") + path);
}

void MainWindow::loadPresetFromFile()
{
    const QString path = QFileDialog::getOpenFileName(this, tr("Load session preset"), QString(),
                                                      tr("Preset JSON (*.json)"));
    if (path.isEmpty())
    {
        return;
    }

    QFile f(path);
    if (!f.open(QIODevice::ReadOnly))
    {
        QMessageBox::warning(this, tr("Preset load failed"), tr("Cannot open preset file."));
        return;
    }
    const QByteArray data = f.readAll();
    f.close();

    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(data, &err);
    if (err.error != QJsonParseError::NoError || !doc.isObject())
    {
        QMessageBox::warning(this, tr("Preset load failed"), tr("Invalid preset JSON."));
        return;
    }

    const QJsonObject root = doc.object();
    SessionPreset preset;
    preset.config_path = root.value(QStringLiteral("config_path")).toString(m_connection->configPath());
    const QJsonArray names = root.value(QStringLiteral("selected_joint_names")).toArray();
    for (const auto &n : names)
    {
        if (n.isString())
        {
            preset.selected_joint_names << n.toString();
        }
    }
    preset.trajectory_mode = root.value(QStringLiteral("trajectory_mode")).toInt(static_cast<int>(TrajectoryMode::Sin));
    preset.logging_enabled = root.value(QStringLiteral("logging_enabled")).toBool(true);

    applyPreset(preset);
    appendLog(QStringLiteral("loaded preset <- ") + path);
}

void MainWindow::exportOffsetsXml()
{
    const QString path = QFileDialog::getSaveFileName(this, tr("Export joint offsets XML"),
                                                      QStringLiteral("joint-offsets.xml"), tr("XML file (*.xml)"));
    if (path.isEmpty())
    {
        return;
    }
    exportOffsetsXmlToPath(path, true);
}

void MainWindow::exportOffsetsXmlToPath(const QString &path, bool silent)
{
    const TelemetryFrame frame = m_worker->snapshot();
    const std::vector<JointInfo> joints = m_worker->joints();
    if (joints.empty())
    {
        QMessageBox::information(this, tr("Export XML"), tr("No joints available. Connect first."));
        return;
    }

    QMap<QString, double> actual_by_name;
    for (const auto &jt : frame.joints)
    {
        actual_by_name.insert(QString::fromStdString(jt.name), jt.actual_deg);
    }

    QStringList names;
    QList<double> mins;
    QList<double> maxs;
    QList<double> homing;
    names.reserve(static_cast<int>(joints.size()));
    mins.reserve(static_cast<int>(joints.size()));
    maxs.reserve(static_cast<int>(joints.size()));
    homing.reserve(static_cast<int>(joints.size()));

    for (const auto &j : joints)
    {
        const QString name = QString::fromStdString(j.name);
        names << name;
        mins << j.min_limit_deg;
        maxs << j.max_limit_deg;
        homing << actual_by_name.value(name, 0.0);
    }

    QFile f(path);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text))
    {
        QMessageBox::warning(this, tr("Export XML failed"), tr("Cannot write XML file."));
        return;
    }

    QTextStream out(&f);
    out << "<group name=\"joint_offsets\">\n";
    out << "  <param name=\"motor_names\">" << tupleQuoted(names) << "</param>\n";
    out << "  <param name=\"position_min\">" << tupleNumbers(mins, 3) << "</param>\n";
    out << "  <param name=\"position_max\">" << tupleNumbers(maxs, 3) << "</param>\n";
    out << "  <param name=\"homing\">" << tupleNumbers(homing, 3) << "</param>\n";
    out << "</group>\n";
    f.close();

    appendLog(QStringLiteral("exported XML -> ") + path);
    if (!silent)
    {
        QMessageBox::information(this, tr("Homing Stored"), tr("Joint offsets exported to:\n%1").arg(path));
    }
}

} // namespace actuator_test::gui
