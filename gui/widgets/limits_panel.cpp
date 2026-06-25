// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/limits_panel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>

namespace actuator_test::gui
{

LimitsPanel::LimitsPanel(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel(tr("Backdrive capture (ticked joints):")));
    m_capture_btn = new QPushButton(tr("Start capture"));
    m_capture_btn->setCheckable(true);
    layout->addWidget(m_capture_btn);

    layout->addWidget(new QLabel(tr("Or set explicit limits:")));
    auto *form = new QFormLayout();
    m_joint_combo = new QComboBox();
    form->addRow(tr("Joint:"), m_joint_combo);
    m_min_spin = new QDoubleSpinBox();
    m_min_spin->setRange(-3600000.0, 3600000.0);
    m_min_spin->setSuffix(tr(" deg"));
    m_max_spin = new QDoubleSpinBox();
    m_max_spin->setRange(-3600000.0, 3600000.0);
    m_max_spin->setSuffix(tr(" deg"));
    form->addRow(tr("Min:"), m_min_spin);
    form->addRow(tr("Max:"), m_max_spin);
    layout->addLayout(form);

    auto *apply = new QPushButton(tr("Apply limits"));
    m_reset_btn = new QPushButton(tr("Reset limits (current position)"));
    layout->addWidget(apply);
    layout->addWidget(m_reset_btn);
    layout->addStretch(1);

    connect(m_capture_btn, &QPushButton::toggled, this, [this](bool on) {
        m_capturing = on;
        m_capture_btn->setText(on ? tr("Finish capture") : tr("Start capture"));
        emit captureToggled(on);
    });
    connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int idx) {
        if (idx >= 0 && static_cast<std::size_t>(idx) < m_joints.size())
        {
            m_min_spin->setValue(m_joints[static_cast<std::size_t>(idx)].min_limit_deg);
            m_max_spin->setValue(m_joints[static_cast<std::size_t>(idx)].max_limit_deg);
        }
    });
    connect(apply, &QPushButton::clicked, this,
            [this] { emit setLimitsRequested(currentJoint(), m_min_spin->value(), m_max_spin->value()); });
    connect(m_reset_btn, &QPushButton::clicked, this, [this] { emit resetLimitsRequested(currentJoint()); });

    setEnabled(false);
}

void LimitsPanel::setJoints(const std::vector<JointInfo> &joints)
{
    m_joints = joints;
    const int prev = m_joint_combo->currentIndex();
    m_joint_combo->blockSignals(true);
    m_joint_combo->clear();
    for (const auto &j : joints)
    {
        m_joint_combo->addItem(QString::fromStdString(j.name));
    }
    if (prev >= 0 && prev < m_joint_combo->count())
    {
        m_joint_combo->setCurrentIndex(prev);
    }
    m_joint_combo->blockSignals(false);
    if (!joints.empty())
    {
        const std::size_t i = currentJoint();
        m_min_spin->setValue(m_joints[i].min_limit_deg);
        m_max_spin->setValue(m_joints[i].max_limit_deg);
    }
}

void LimitsPanel::setState(ControllerState state)
{
    const bool idle_or_capturing =
        (state == ControllerState::Connected || state == ControllerState::Capturing);
    setEnabled(idle_or_capturing);
}

void LimitsPanel::updateLiveLimits(const std::vector<JointInfo> &joints)
{
    if (joints.size() != m_joints.size())
    {
        return; // Combo not yet in sync; setJoints() will refresh.
    }
    // Only auto-fill spin boxes during capturing mode
    if (!m_capturing)
    {
        return;
    }
    for (std::size_t i = 0; i < joints.size(); ++i)
    {
        m_joints[i].min_limit_deg = joints[i].min_limit_deg;
        m_joints[i].max_limit_deg = joints[i].max_limit_deg;
    }
    const std::size_t cur = currentJoint();
    if (cur >= m_joints.size())
    {
        return;
    }
    // Only auto-fill spin boxes that the user is not actively editing.
    if (!m_min_spin->hasFocus())
    {
        m_min_spin->blockSignals(true);
        m_min_spin->setValue(m_joints[cur].min_limit_deg);
        m_min_spin->blockSignals(false);
    }
    if (!m_max_spin->hasFocus())
    {
        m_max_spin->blockSignals(true);
        m_max_spin->setValue(m_joints[cur].max_limit_deg);
        m_max_spin->blockSignals(false);
    }
}

std::size_t LimitsPanel::currentJoint() const
{
    const int idx = m_joint_combo->currentIndex();
    return idx < 0 ? 0 : static_cast<std::size_t>(idx);
}

} // namespace actuator_test::gui
