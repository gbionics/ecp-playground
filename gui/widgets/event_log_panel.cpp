// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "event_log_panel.hpp"

#include <QFile>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>
#include <QString>
#include <QTableWidget>
#include <QTextStream>
#include <QVBoxLayout>

namespace actuator_test::gui {

const char *to_string(EventType type) noexcept {
  switch (type) {
  case EventType::Info:
    return "Info";
  case EventType::Warning:
    return "Warning";
  case EventType::Error:
    return "Error";
  case EventType::Fault:
    return "Fault";
  case EventType::LimitViolation:
    return "Limit Violation";
  case EventType::StateChange:
    return "State Change";
  case EventType::HardwareStatus:
    return "Hardware Status";
  }
  return "Unknown";
}

EventLogPanel::EventLogPanel(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);

  // Button toolbar
  auto *toolbar = new QHBoxLayout();
  m_clear_btn = new QPushButton(tr("Clear Log"));
  connect(m_clear_btn, &QPushButton::clicked, this, &EventLogPanel::onClearLog);
  toolbar->addWidget(m_clear_btn);

  m_export_btn = new QPushButton(tr("Export to CSV"));
  connect(m_export_btn, &QPushButton::clicked, this,
          &EventLogPanel::onExportLog);
  toolbar->addWidget(m_export_btn);

  toolbar->addStretch();
  layout->addLayout(toolbar);

  // Event table
  m_log_table = new QTableWidget(0, 5);
  m_log_table->setHorizontalHeaderLabels(
      {tr("Time"), tr("Type"), tr("Joint"), tr("Message"), tr("Value")});
  m_log_table->horizontalHeader()->setSectionResizeMode(
      0, QHeaderView::ResizeToContents);
  m_log_table->horizontalHeader()->setSectionResizeMode(
      1, QHeaderView::ResizeToContents);
  m_log_table->horizontalHeader()->setSectionResizeMode(
      2, QHeaderView::ResizeToContents);
  m_log_table->horizontalHeader()->setSectionResizeMode(3,
                                                        QHeaderView::Stretch);
  m_log_table->horizontalHeader()->setSectionResizeMode(
      4, QHeaderView::ResizeToContents);
  m_log_table->setEditTriggers(QAbstractItemView::NoEditTriggers);
  m_log_table->setSelectionBehavior(QAbstractItemView::SelectRows);
  m_log_table->setAlternatingRowColors(true);
  layout->addWidget(m_log_table);

  setLayout(layout);
}

void EventLogPanel::logEvent(const SystemEvent &event) {
  if (m_events.size() >= MAX_EVENTS) {
    m_events.erase(m_events.begin());
  }
  m_events.push_back(event);
  addEventRow(event);

  // Auto-scroll to bottom
  m_log_table->scrollToBottom();
}

void EventLogPanel::logEvents(const std::vector<SystemEvent> &events) {
  for (const auto &event : events) {
    logEvent(event);
  }
}

void EventLogPanel::clearLog() {
  m_events.clear();
  m_log_table->setRowCount(0);
}

void EventLogPanel::exportToFile(const QString &path) {
  QFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
    return;

  QTextStream stream(&file);
  stream << "Time,Type,Joint,Message,Value\n";

  for (const auto &event : m_events) {
    stream << event.timestamp.toString("yyyy-MM-dd HH:mm:ss.zzz") << ","
           << to_string(event.type) << "," << event.joint_name << ","
           << event.message << "," << event.associated_value << "\n";
  }

  file.close();
}

void EventLogPanel::onClearLog() { clearLog(); }

void EventLogPanel::onExportLog() {
  QString fileName = QFileDialog::getSaveFileName(
      this, tr("Export Event Log"), "", tr("CSV Files (*.csv);;All Files (*)"));
  if (!fileName.isEmpty()) {
    exportToFile(fileName);
  }
}

void EventLogPanel::onFilterByJoint(const QString &name) {
  // TODO: Implement filtering
}

void EventLogPanel::refreshTable() {
  m_log_table->setRowCount(0);
  for (const auto &event : m_events) {
    addEventRow(event);
  }
}

void EventLogPanel::addEventRow(const SystemEvent &event) {
  int row = m_log_table->rowCount();
  m_log_table->insertRow(row);

  auto *time_item =
      new QTableWidgetItem(event.timestamp.toString("HH:mm:ss.zzz"));
  auto *type_item = new QTableWidgetItem(to_string(event.type));
  auto *joint_item = new QTableWidgetItem(event.joint_name);
  auto *message_item = new QTableWidgetItem(event.message);
  auto *value_item =
      new QTableWidgetItem(QString::number(event.associated_value, 'f', 2));

  type_item->setBackground(eventTypeColor(event.type));
  if (event.type == EventType::Error || event.type == EventType::Fault) {
    type_item->setForeground(QColor(255, 255, 255));
  }

  m_log_table->setItem(row, 0, time_item);
  m_log_table->setItem(row, 1, type_item);
  m_log_table->setItem(row, 2, joint_item);
  m_log_table->setItem(row, 3, message_item);
  m_log_table->setItem(row, 4, value_item);
}

QColor EventLogPanel::eventTypeColor(EventType type) const {
  switch (type) {
  case EventType::Info:
    return QColor(230, 245, 255);
  case EventType::Warning:
    return QColor(255, 250, 200);
  case EventType::Error:
    return QColor(255, 200, 200);
  case EventType::Fault:
    return QColor(220, 20, 60);
  case EventType::LimitViolation:
    return QColor(255, 200, 150);
  case EventType::StateChange:
    return QColor(200, 230, 200);
  case EventType::HardwareStatus:
    return QColor(240, 240, 240);
  }
  return QColor(255, 255, 255);
}

QString EventLogPanel::eventTypeIcon(EventType type) const {
  switch (type) {
  case EventType::Info:
    return "ℹ";
  case EventType::Warning:
    return "⚠";
  case EventType::Error:
    return "✕";
  case EventType::Fault:
    return "⚡";
  case EventType::LimitViolation:
    return "◆";
  case EventType::StateChange:
    return "→";
  case EventType::HardwareStatus:
    return "◆";
  }
  return "";
}

} // namespace actuator_test::gui
