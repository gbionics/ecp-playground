// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "enhanced_limits_panel.hpp"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QString>

#include <algorithm>

namespace actuator_test::gui
{

EnhancedLimitsPanel::EnhancedLimitsPanel(QWidget *parent)
    : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    // Joint selector
    auto *selector_layout = new QHBoxLayout();
    selector_layout->addWidget(new QLabel(tr("Joint:")));
    m_joint_combo = new QComboBox();
    connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &EnhancedLimitsPanel::onJointChanged);
    selector_layout->addWidget(m_joint_combo);
    selector_layout->addStretch();
    layout->addLayout(selector_layout);

    // Auto capture mode
    auto *capture_group = new QGroupBox(tr("Mode 1: Auto Capture"));
    auto *capture_layout = new QVBoxLayout(capture_group);
    m_capture_btn = new QPushButton(tr("Start Backdrive Capture"));
    m_capture_btn->setCheckable(true);
    connect(m_capture_btn, &QPushButton::toggled, this, [this](bool checked) {
        m_capture_btn->setText(checked ? tr("Stop Capture and Apply") : tr("Start Backdrive Capture"));
        emit captureToggled(checked);
    });
    capture_layout->addWidget(m_capture_btn);
    m_auto_info_label = new QLabel(
        tr("Move the actuator to physical minimum and maximum while capture is ON.\n"
           "Press stop to automatically apply captured min/max."));
    capture_layout->addWidget(m_auto_info_label);
    layout->addWidget(capture_group);

    // Manual override mode
    auto *hard_limits_group = new QGroupBox(tr("Mode 2: Manual Override"));
    auto *hard_limits_layout = new QHBoxLayout(hard_limits_group);
    hard_limits_layout->addWidget(new QLabel(tr("Min:")));
    m_hard_min_spin = new QDoubleSpinBox();
    m_hard_min_spin->setRange(-1000, 1000);
    m_hard_min_spin->setDecimals(2);
    m_hard_min_spin->setSuffix(tr(" deg"));
    hard_limits_layout->addWidget(m_hard_min_spin);
    hard_limits_layout->addWidget(new QLabel(tr("Max:")));
    m_hard_max_spin = new QDoubleSpinBox();
    m_hard_max_spin->setRange(-1000, 1000);
    m_hard_max_spin->setDecimals(2);
    m_hard_max_spin->setSuffix(tr(" deg"));
    hard_limits_layout->addWidget(m_hard_max_spin);

    m_apply_limits_btn = new QPushButton(tr("Apply Min/Max"));
    connect(m_apply_limits_btn, &QPushButton::clicked, this, &EnhancedLimitsPanel::onApplyLimitsClicked);
    hard_limits_layout->addWidget(m_apply_limits_btn);

    m_reset_btn = new QPushButton(tr("Reset"));
    connect(m_reset_btn, &QPushButton::clicked, this,
            [this] { emit resetLimitsRequested(currentJoint()); });
    hard_limits_layout->addWidget(m_reset_btn);
    layout->addWidget(hard_limits_group);

    m_manual_info_label = new QLabel(tr("Enter both min and max and press Apply Min/Max."));
    layout->addWidget(m_manual_info_label);
    layout->addStretch();
    setLayout(layout);
}

void EnhancedLimitsPanel::setJoints(const std::vector<JointInfo> &joints)
{
    m_joints = joints;
    m_joint_combo->clear();
    for (const auto &j : joints)
    {
        m_joint_combo->addItem(QString::fromStdString(j.name));
    }
}

void EnhancedLimitsPanel::setState(ControllerState state)
{
    bool enabled = (state != ControllerState::Disconnected);
    m_capture_btn->setEnabled(enabled);
    m_apply_limits_btn->setEnabled(enabled);
    m_reset_btn->setEnabled(enabled);
}

void EnhancedLimitsPanel::updateLiveLimits(const std::vector<JointInfo> &joints,
                                           const std::vector<JointTelemetry> &telemetry)
{
    if (joints.empty() || telemetry.empty())
        return;

    m_joints = joints;
    m_telemetry = telemetry;

    int idx = m_joint_combo->currentIndex();
    if (idx < 0 || idx >= static_cast<int>(joints.size()))
        idx = 0;

    const auto &joint_info = joints[idx];
    const auto &telem = telemetry[idx];

    // Avoid overwriting while the operator is typing values.
    if (!m_hard_min_spin->hasFocus())
    {
        m_hard_min_spin->setValue(joint_info.min_limit_deg);
    }
    if (!m_hard_max_spin->hasFocus())
    {
        m_hard_max_spin->setValue(joint_info.max_limit_deg);
    }

    Q_UNUSED(telem);
}

void EnhancedLimitsPanel::onJointChanged(int index)
{
    if (index >= 0 && index < static_cast<int>(m_joints.size()))
    {
        const auto &joint_info = m_joints[static_cast<std::size_t>(index)];
        if (!m_hard_min_spin->hasFocus())
        {
            m_hard_min_spin->setValue(joint_info.min_limit_deg);
        }
        if (!m_hard_max_spin->hasFocus())
        {
            m_hard_max_spin->setValue(joint_info.max_limit_deg);
        }
    }
}

void EnhancedLimitsPanel::onApplyLimitsClicked()
{
    double min_deg = m_hard_min_spin->value();
    double max_deg = m_hard_max_spin->value();
    if (min_deg > max_deg)
    {
        std::swap(min_deg, max_deg);
        m_hard_min_spin->setValue(min_deg);
        m_hard_max_spin->setValue(max_deg);
    }
    emit setLimitsRequested(currentJoint(), min_deg, max_deg);
}


std::size_t EnhancedLimitsPanel::currentJoint() const
{
    int idx = m_joint_combo->currentIndex();
    return (idx >= 0) ? static_cast<std::size_t>(idx) : 0;
}

} // namespace actuator_test::gui
