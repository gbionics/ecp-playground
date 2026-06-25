// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Soft-limit definition: either backdrive-capture the travelled envelope for
// the selected joints, or type explicit min/max for one joint.

#pragma once

#include "core/controller_worker.hpp"

#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
class QPushButton;
QT_END_NAMESPACE

namespace actuator_test::gui
{

class LimitsPanel : public QWidget
{
    Q_OBJECT

public:
    explicit LimitsPanel(QWidget *parent = nullptr);

    void setJoints(const std::vector<JointInfo> &joints);
    void setState(ControllerState state);

    /// Reflect live captured limits in the spin boxes without disturbing user edits.
    void updateLiveLimits(const std::vector<JointInfo> &joints);

signals:
    /// Begin (true) or finish (false) a backdrive capture on the panel's joints.
    void captureToggled(bool start);
    void setLimitsRequested(std::size_t joint, double min_deg, double max_deg);
    void resetLimitsRequested(std::size_t joint);

private:
    std::size_t currentJoint() const;

    QPushButton *m_capture_btn = nullptr;
    QPushButton *m_reset_btn = nullptr;
    QComboBox *m_joint_combo = nullptr;
    QDoubleSpinBox *m_min_spin = nullptr;
    QDoubleSpinBox *m_max_spin = nullptr;
    std::vector<JointInfo> m_joints;
    bool m_capturing = false;
};

} // namespace actuator_test::gui
