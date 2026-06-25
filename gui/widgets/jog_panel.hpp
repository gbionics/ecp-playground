// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Manual control of a single joint: press-and-hold jog, plus one-shot moves to
// the captured centre / minimum (homing).

#pragma once

#include "core/controller_worker.hpp"

#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
class QDoubleSpinBox;
QT_END_NAMESPACE

namespace actuator_test::gui
{

class JogPanel : public QWidget
{
    Q_OBJECT

public:
    explicit JogPanel(QWidget *parent = nullptr);

    void setJoints(const std::vector<JointInfo> &joints);
    void setEnabledControls(bool enabled);

    /// Refresh cached limit values (used for centre/home targets) without rebuilding the combo.
    void updateLiveLimits(const std::vector<JointInfo> &joints);

signals:
    void jogRequested(std::size_t joint, double velocity_deg_s);
    void goToRequested(std::size_t joint, double target_deg, double speed_deg_s);
    void stopRequested();

private:
    std::size_t currentJoint() const;

    QComboBox *m_joint_combo = nullptr;
    QDoubleSpinBox *m_speed_spin = nullptr;
    std::vector<JointInfo> m_joints;
};

} // namespace actuator_test::gui
