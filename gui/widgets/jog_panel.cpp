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

namespace actuator_test::gui
{

JogPanel::JogPanel(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *form = new QFormLayout();
    m_joint_combo = new QComboBox();
    form->addRow(tr("Joint:"), m_joint_combo);

    m_speed_spin = new QDoubleSpinBox();
    m_speed_spin->setRange(0.1, 360.0);
    m_speed_spin->setValue(30.0);
    m_speed_spin->setSuffix(tr(" deg/s"));
    form->addRow(tr("Jog speed:"), m_speed_spin);
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
    auto *minus = new QPushButton(tr("- Jog"));
    auto *stop = new QPushButton(tr("Stop"));
    auto *plus = new QPushButton(tr("Jog +"));
    jog_row->addWidget(minus);
    jog_row->addWidget(stop);
    jog_row->addWidget(plus);
    layout->addLayout(jog_row);

    auto *home_row = new QHBoxLayout();
    auto *centre = new QPushButton(tr("Go to centre"));
    auto *home = new QPushButton(tr("Home (min)"));
    home_row->addWidget(centre);
    home_row->addWidget(home);
    layout->addLayout(home_row);
    layout->addStretch(1);

    // Press-and-hold jogging: jog while held, stop on release.
    connect(minus, &QPushButton::pressed, this,
            [this] { emit jogRequested(currentJoint(), -m_speed_spin->value()); });
    connect(minus, &QPushButton::released, this, [this] { emit jogRequested(currentJoint(), 0.0); });
    connect(plus, &QPushButton::pressed, this,
            [this] { emit jogRequested(currentJoint(), m_speed_spin->value()); });
    connect(plus, &QPushButton::released, this, [this] { emit jogRequested(currentJoint(), 0.0); });
    connect(stop, &QPushButton::clicked, this, &JogPanel::stopRequested);
    connect(slow, &QPushButton::clicked, this, [this] { m_speed_spin->setValue(10.0); });
    connect(medium, &QPushButton::clicked, this, [this] { m_speed_spin->setValue(30.0); });
    connect(fast, &QPushButton::clicked, this, [this] { m_speed_spin->setValue(60.0); });

    connect(centre, &QPushButton::clicked, this, [this] {
        const std::size_t j = currentJoint();
        if (j < m_joints.size())
        {
            const double mid = 0.5 * (m_joints[j].min_limit_deg + m_joints[j].max_limit_deg);
            emit goToRequested(j, mid, m_speed_spin->value());
        }
    });
    connect(home, &QPushButton::clicked, this, [this] {
        const std::size_t j = currentJoint();
        if (j < m_joints.size())
        {
            emit goToRequested(j, m_joints[j].min_limit_deg, m_speed_spin->value());
        }
    });

    setEnabledControls(false);
}

void JogPanel::setJoints(const std::vector<JointInfo> &joints)
{
    m_joints = joints;
    const QString current = m_joint_combo->currentText();
    m_joint_combo->clear();
    for (const auto &j : joints)
    {
        m_joint_combo->addItem(QString::fromStdString(j.name));
    }
    const int idx = m_joint_combo->findText(current);
    if (idx >= 0)
    {
        m_joint_combo->setCurrentIndex(idx);
    }
}

void JogPanel::setEnabledControls(bool enabled)
{
    setEnabled(enabled);
}

void JogPanel::updateLiveLimits(const std::vector<JointInfo> &joints)
{
    if (joints.size() != m_joints.size())
    {
        return; // Combo not yet in sync; setJoints() will refresh.
    }
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        m_joints[i].min_limit_deg = joints[i].min_limit_deg;
        m_joints[i].max_limit_deg = joints[i].max_limit_deg;
    }
}

std::size_t JogPanel::currentJoint() const
{
    const int idx = m_joint_combo->currentIndex();
    return idx < 0 ? 0 : static_cast<std::size_t>(idx);
}

} // namespace actuator_test::gui
