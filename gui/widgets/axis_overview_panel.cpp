// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "axis_overview_panel.hpp"

#include <QFont>
#include <QFontMetrics>
#include <QGridLayout>
#include <QPaintEvent>
#include <QPainter>
#include <QString>

namespace actuator_test::gui {

// ---------------------------------------------------------------------------
//  AxisStatusWidget
// ---------------------------------------------------------------------------

AxisStatusWidget::AxisStatusWidget(const QString &name, QWidget *parent)
    : QWidget(parent), m_axis_name(name) {
  setMinimumSize(150, 120);
  setMaximumSize(200, 150);
}

void AxisStatusWidget::setTelemetry(const JointTelemetry &telem) {
  m_telem = telem;
  update();
}

void AxisStatusWidget::paintEvent(QPaintEvent *event) {
  QPainter p(this);
  p.setRenderHint(QPainter::Antialiasing);

  // Background
  QColor bg(26, 28, 32);
  QColor border(50, 55, 65);

  if (m_telem.fault) {
    bg = QColor(220, 38, 38).lighter(50);
  } else if (m_telem.limit_violation) {
    bg = QColor(234, 179, 8).lighter(60);
  } else if (m_telem.cia402_state == CiA402State::OperationEnabled) {
    bg = QColor(34, 197, 94).lighter(80);
  }

  p.fillRect(rect(), bg);
  p.setPen(QPen(border, 2));
  p.drawRect(0, 0, width() - 1, height() - 1);

  // Title
  p.setFont(QFont("Arial", 10, QFont::Bold));
  p.setPen(QColor(200, 200, 200));
  p.drawText(5, 5, width() - 10, 20, Qt::AlignCenter, m_axis_name);

  // Position
  p.setFont(QFont("Arial", 9));
  p.drawText(5, 25, width() - 10, 18, Qt::AlignLeft,
             QString::number(m_telem.actual_deg, 'f', 1) + " deg");

  // Status indicator
  QColor status_color = m_telem.fault             ? QColor(220, 38, 38)
                        : m_telem.limit_violation ? QColor(234, 179, 8)
                        : m_telem.cia402_state == CiA402State::OperationEnabled
                            ? QColor(34, 197, 94)
                            : QColor(150, 150, 150);

  p.fillRect(width() - 22, 5, 17, 17, status_color);
  p.setPen(QPen(QColor(0, 0, 0), 1));
  p.drawRect(width() - 22, 5, 17, 17);

  // Temperature indicator if available
  if (m_telem.drive_temp_c >= 0) {
    QString temp_text = QString::number(m_telem.drive_temp_c) + "°C";
    p.setFont(QFont("Arial", 8));

    QColor temp_color = m_telem.drive_temp_c > 70   ? QColor(220, 38, 38)
                        : m_telem.drive_temp_c > 55 ? QColor(234, 179, 8)
                                                    : QColor(100, 150, 255);
    p.setPen(temp_color);
    p.drawText(5, height() - 22, width() - 10, 18, Qt::AlignCenter, temp_text);
  }

  // Velocity indicator
  p.setFont(QFont("Arial", 8));
  p.setPen(QColor(150, 200, 255));
  p.drawText(5, height() - 40, width() - 10, 16, Qt::AlignCenter,
             "V:" + QString::number(m_telem.velocity_deg_s, 'f', 1) + " deg/s");
}

// ---------------------------------------------------------------------------
//  AxisOverviewPanel
// ---------------------------------------------------------------------------

AxisOverviewPanel::AxisOverviewPanel(QWidget *parent) : QWidget(parent) {
  setMinimumHeight(200);
  auto *layout = new QGridLayout(this);
  layout->setSpacing(8);
  layout->setContentsMargins(4, 4, 4, 4);
  setLayout(layout);
}

void AxisOverviewPanel::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;

  // Clear existing widgets
  while (!m_status_widgets.empty()) {
    auto w = m_status_widgets.back();
    m_status_widgets.pop_back();
    w->deleteLater();
  }

  // Create new status widgets
  auto *layout = qobject_cast<QGridLayout *>(this->layout());
  for (size_t i = 0; i < joints.size(); ++i) {
    auto *widget = new AxisStatusWidget(QString::fromStdString(joints[i].name));
    m_status_widgets.push_back(widget);
    layout->addWidget(widget, static_cast<int>(i) / 4, static_cast<int>(i) % 4);
  }
}

void AxisOverviewPanel::updateTelemetry(
    const std::vector<JointTelemetry> &joints) {
  for (size_t i = 0; i < joints.size() && i < m_status_widgets.size(); ++i) {
    m_status_widgets[i]->setTelemetry(joints[i]);
  }
}

} // namespace actuator_test::gui
