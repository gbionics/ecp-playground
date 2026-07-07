// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/jog_panel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>

namespace actuator_test::gui {

JogPanel::JogPanel(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);

  auto *form = new QFormLayout();
  m_joint_combo = new QComboBox();
  m_joint_combo->setAccessibleName(tr("Joint selection"));
  auto *joint_label = new QLabel(tr("&Joint:"));
  joint_label->setBuddy(m_joint_combo);
  form->addRow(joint_label, m_joint_combo);

  m_speed_spin = new QDoubleSpinBox();
  m_speed_spin->setRange(0.1, 360.0);
  m_speed_spin->setValue(30.0);
  m_speed_spin->setSuffix(tr(" deg/s"));
  m_speed_spin->setAccessibleName(tr("Jog speed in degrees per second"));
  auto *speed_label = new QLabel(tr("Jog &speed:"));
  speed_label->setBuddy(m_speed_spin);
  form->addRow(speed_label, m_speed_spin);
  layout->addLayout(form);

  auto *preset_row = new QHBoxLayout();
  preset_row->addWidget(new QLabel(tr("Speed presets:")));
  auto *slow = new QPushButton(tr("10"));
  auto *medium = new QPushButton(tr("30"));
  auto *fast = new QPushButton(tr("60"));
  slow->setToolTip(tr("Set jog speed to 10 deg/s"));
  medium->setToolTip(tr("Set jog speed to 30 deg/s"));
  fast->setToolTip(tr("Set jog speed to 60 deg/s"));
  preset_row->addWidget(slow);
  preset_row->addWidget(medium);
  preset_row->addWidget(fast);
  preset_row->addStretch(1);
  layout->addLayout(preset_row);

  auto *jog_row = new QHBoxLayout();
  auto *minus = new QPushButton(tr("\u2212 Jog"));
  auto *stop = new QPushButton(tr("&Stop"));
  auto *plus = new QPushButton(tr("Jog +"));
  minus->setToolTip(tr("Hold to jog the joint in the negative direction."));
  minus->setAccessibleName(tr("Jog negative"));
  stop->setToolTip(tr("Stop jogging and hold the current position."));
  stop->setAccessibleName(tr("Stop motion"));
  plus->setToolTip(tr("Hold to jog the joint in the positive direction."));
  plus->setAccessibleName(tr("Jog positive"));
  jog_row->addWidget(minus);
  jog_row->addWidget(stop);
  jog_row->addWidget(plus);
  layout->addLayout(jog_row);

  auto *home_row = new QHBoxLayout();
  auto *centre = new QPushButton(tr("Go to &centre"));
  auto *home = new QPushButton(tr("&Home (min)"));
  centre->setToolTip(
      tr("Move smoothly to the midpoint of the captured range and hold."));
  centre->setAccessibleName(tr("Go to centre"));
  home->setToolTip(tr("Move smoothly to the minimum limit and hold."));
  home->setAccessibleName(tr("Home to minimum"));
  home_row->addWidget(centre);
  home_row->addWidget(home);
  layout->addLayout(home_row);

  // --- Manual position reference: drive to an absolute angle and hold. ---
  auto *target_row = new QHBoxLayout();
  auto *target_label = new QLabel(tr("&Target:"));
  target_row->addWidget(target_label);
  m_target_spin = new QDoubleSpinBox();
  m_target_spin->setRange(-3600000.0, 3600000.0);
  m_target_spin->setDecimals(2);
  m_target_spin->setSuffix(tr(" deg"));
  m_target_spin->setAccessibleName(tr("Target angle in degrees"));
  target_label->setBuddy(m_target_spin);
  target_row->addWidget(m_target_spin, 1);
  auto *go_hold = new QPushButton(tr("&Go && Hold"));
  go_hold->setToolTip(
      tr("Move to the target angle and hold there until Stop is pressed."));
  go_hold->setAccessibleName(tr("Go to target and hold"));
  target_row->addWidget(go_hold);
  layout->addLayout(target_row);
  layout->addStretch(1);

  connect(go_hold, &QPushButton::clicked, this, [this] {
    emit goToRequested(currentJoint(), m_target_spin->value(),
                       m_speed_spin->value());
  });

  // Press-and-hold jogging: jog while held, stop on release.
  connect(minus, &QPushButton::pressed, this, [this] {
    emit jogRequested(currentJoint(), -m_speed_spin->value());
  });
  connect(minus, &QPushButton::released, this,
          [this] { emit jogRequested(currentJoint(), 0.0); });
  connect(plus, &QPushButton::pressed, this,
          [this] { emit jogRequested(currentJoint(), m_speed_spin->value()); });
  connect(plus, &QPushButton::released, this,
          [this] { emit jogRequested(currentJoint(), 0.0); });
  connect(stop, &QPushButton::clicked, this, &JogPanel::stopRequested);
  connect(slow, &QPushButton::clicked, this,
          [this] { m_speed_spin->setValue(10.0); });
  connect(medium, &QPushButton::clicked, this,
          [this] { m_speed_spin->setValue(30.0); });
  connect(fast, &QPushButton::clicked, this,
          [this] { m_speed_spin->setValue(60.0); });

  connect(centre, &QPushButton::clicked, this, [this] {
    const std::size_t j = currentJoint();
    if (j < m_joints.size()) {
      const double mid =
          0.5 * (m_joints[j].min_limit_deg + m_joints[j].max_limit_deg);
      emit goToRequested(j, mid, m_speed_spin->value());
    }
  });
  connect(home, &QPushButton::clicked, this, [this] {
    const std::size_t j = currentJoint();
    if (j < m_joints.size()) {
      emit goToRequested(j, m_joints[j].min_limit_deg, m_speed_spin->value());
    }
  });

  connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { syncTargetRange(true); });

  setEnabledControls(false);
}

void JogPanel::setJoints(const std::vector<JointInfo> &joints) {
  m_joints = joints;
  const QString current = m_joint_combo->currentText();
  m_joint_combo->clear();
  for (const auto &j : joints) {
    m_joint_combo->addItem(QString::fromStdString(j.name));
  }
  const int idx = m_joint_combo->findText(current);
  if (idx >= 0) {
    m_joint_combo->setCurrentIndex(idx);
  }
  syncTargetRange();
}

void JogPanel::setEnabledControls(bool enabled) { setEnabled(enabled); }

void JogPanel::updateLiveLimits(const std::vector<JointInfo> &joints) {
  if (joints.size() != m_joints.size()) {
    return; // Combo not yet in sync; setJoints() will refresh.
  }
  for (std::size_t i = 0; i < joints.size(); ++i) {
    m_joints[i].min_limit_deg = joints[i].min_limit_deg;
    m_joints[i].max_limit_deg = joints[i].max_limit_deg;
  }
  syncTargetRange(false);
}

std::size_t JogPanel::currentJoint() const {
  const int idx = m_joint_combo->currentIndex();
  return idx < 0 ? 0 : static_cast<std::size_t>(idx);
}

void JogPanel::syncTargetRange(bool reset_value) {
  if (!m_target_spin) {
    return;
  }
  const std::size_t j = currentJoint();
  if (j >= m_joints.size()) {
    return;
  }
  const double lo = m_joints[j].min_limit_deg;
  const double hi = m_joints[j].max_limit_deg;
  m_target_spin->setRange(std::min(lo, hi), std::max(lo, hi));
  if (reset_value && !m_target_spin->hasFocus()) {
    m_target_spin->setValue(0.5 * (lo + hi));
  }
}

} // namespace actuator_test::gui
