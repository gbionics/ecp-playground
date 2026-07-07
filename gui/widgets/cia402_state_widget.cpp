// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "cia402_state_widget.hpp"

#include <QColor>
#include <QPainter>
#include <QPainterPath>
#include <QResizeEvent>

namespace actuator_test::gui {

CiA402StateWidget::CiA402StateWidget(QWidget *parent) : QWidget(parent) {
  setMinimumHeight(240);
  layoutStates();
}

void CiA402StateWidget::setState(CiA402State state, bool fault) {
  if (m_current_state != state || m_fault != fault) {
    m_current_state = state;
    m_fault = fault;
    update();
  }
}

void CiA402StateWidget::layoutStates() {
  m_state_boxes.clear();

  // Create state boxes in a roughly circular layout
  // Row 1: NotReadyToSwitchOn, SwitchedOnDisabled, ReadyToSwitchOn
  m_state_boxes.push_back(
      {"Not Ready", 20, 20, 100, 40, CiA402State::NotReadyToSwitchOn});
  m_state_boxes.push_back({"Switched On\nDisabled", 140, 20, 100, 40,
                           CiA402State::SwitchedOnDisabled});
  m_state_boxes.push_back(
      {"Ready to\nSwitch On", 260, 20, 100, 40, CiA402State::ReadyToSwitchOn});

  // Row 2: SwitchedOn, OperationEnabled
  m_state_boxes.push_back(
      {"Switched On", 140, 100, 100, 40, CiA402State::SwitchedOn});
  m_state_boxes.push_back(
      {"Operation\nEnabled", 260, 100, 100, 40, CiA402State::OperationEnabled});

  // Row 3: QuickStop, FaultReactionActive, Fault
  m_state_boxes.push_back(
      {"Quick Stop", 20, 180, 100, 40, CiA402State::QuickStop});
  m_state_boxes.push_back(
      {"Fault\nReaction", 140, 180, 100, 40, CiA402State::FaultReactionActive});
  m_state_boxes.push_back({"Fault", 260, 180, 100, 40, CiA402State::Fault});
}

CiA402StateWidget::StateBox
CiA402StateWidget::stateAt(CiA402State state) const {
  for (const auto &box : m_state_boxes) {
    if (box.state == state)
      return box;
  }
  return StateBox();
}

QColor CiA402StateWidget::stateColor(CiA402State state, bool active) const {
  if (active) {
    if (state == CiA402State::Fault ||
        state == CiA402State::FaultReactionActive)
      return QColor(220, 38, 38); // Red for fault
    else if (state == CiA402State::OperationEnabled)
      return QColor(34, 197, 94); // Green for enabled
    else if (state == CiA402State::QuickStop)
      return QColor(234, 179, 8); // Yellow for quick stop
    else
      return QColor(59, 130, 246); // Blue for other active states
  } else {
    return QColor(209, 213, 219); // Light gray for inactive
  }
}

void CiA402StateWidget::paintEvent(QPaintEvent *event) {
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  // Draw state boxes
  for (const auto &box : m_state_boxes) {
    bool active = (box.state == m_current_state);
    QColor color = stateColor(box.state, active);

    // Draw box
    painter.fillRect(box.x, box.y, box.w, box.h, color);
    painter.setPen(QPen(active ? QColor(0, 0, 0) : QColor(100, 100, 100), 2));
    painter.drawRect(box.x, box.y, box.w, box.h);

    // Draw text
    painter.setPen(QColor(0, 0, 0));
    painter.setFont(QFont("Arial", active ? 9 : 8));
    painter.drawText(box.x, box.y, box.w, box.h, Qt::AlignCenter, box.name);
  }

  // Draw simple state transition arrows
  QPen arrow_pen(QColor(100, 100, 100), 1);
  painter.setPen(arrow_pen);

  // Not Ready -> Switched On Disabled
  painter.drawLine(120, 40, 140, 40);
  // Switched On Disabled -> Ready to Switch On
  painter.drawLine(240, 40, 260, 40);
  // Ready to Switch On -> Switched On
  painter.drawLine(310, 60, 190, 100);
  // Switched On -> Operation Enabled
  painter.drawLine(240, 120, 260, 120);
  // Operation Enabled -> Switched On (back)
  painter.drawLine(260, 140, 190, 140);
  // Quick Stop -> Fault Reaction
  painter.drawLine(120, 200, 140, 200);
  // Fault Reaction -> Fault
  painter.drawLine(240, 200, 260, 200);

  // Draw fault indicator if active
  if (m_fault) {
    painter.setPen(QPen(QColor(220, 38, 38), 3));
    painter.drawRect(5, 5, width() - 10, height() - 10);
    painter.setFont(QFont("Arial", 10, QFont::Bold));
    painter.setPen(QColor(220, 38, 38));
    painter.drawText(rect(), Qt::AlignTop | Qt::AlignRight, "FAULT");
  }
}

void CiA402StateWidget::resizeEvent(QResizeEvent *event) {
  QWidget::resizeEvent(event);
  layoutStates();
}

} // namespace actuator_test::gui
