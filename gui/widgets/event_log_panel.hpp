// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Event and alarm log panel for system diagnostics and troubleshooting.

#pragma once

#include "core/telemetry.hpp"

#include <QDateTime>
#include <QString>
#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QTableWidget;
class QPushButton;
QT_END_NAMESPACE

namespace actuator_test::gui {

/// Event types for the system event log
enum class EventType {
  Info,
  Warning,
  Error,
  Fault,
  LimitViolation,
  StateChange,
  HardwareStatus,
};

const char *to_string(EventType type) noexcept;

/// System event for logging and analysis
struct SystemEvent {
  QDateTime timestamp;
  EventType type = EventType::Info;
  QString joint_name;
  QString message;
  uint16_t error_code = 0;
  double associated_value = 0.0;
};

/// Event and alarm log panel for comprehensive system monitoring
class EventLogPanel : public QWidget {
  Q_OBJECT

public:
  explicit EventLogPanel(QWidget *parent = nullptr);

  /// Log a system event
  void logEvent(const SystemEvent &event);

  /// Log multiple events at once
  void logEvents(const std::vector<SystemEvent> &events);

  /// Clear the event log
  void clearLog();

  /// Get all logged events
  std::vector<SystemEvent> events() const { return m_events; }

  /// Export log to CSV file
  void exportToFile(const QString &path);

private slots:
  void onClearLog();
  void onExportLog();
  void onFilterByJoint(const QString &name);

private:
  void refreshTable();
  void addEventRow(const SystemEvent &event);
  QColor eventTypeColor(EventType type) const;
  QString eventTypeIcon(EventType type) const;

  QTableWidget *m_log_table = nullptr;
  QPushButton *m_clear_btn = nullptr;
  QPushButton *m_export_btn = nullptr;
  std::vector<SystemEvent> m_events;
  static constexpr int MAX_EVENTS = 10000;
};

} // namespace actuator_test::gui
