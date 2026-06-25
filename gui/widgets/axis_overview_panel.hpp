// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Axis overview dashboard showing status and metrics for all joints at a glance.

#pragma once

#include "core/telemetry.hpp"
#include "core/controller_worker.hpp"

#include <QWidget>
#include <vector>

namespace actuator_test::gui
{

/// Compact status widget for a single axis
class AxisStatusWidget : public QWidget
{
    Q_OBJECT

public:
    explicit AxisStatusWidget(const QString &name, QWidget *parent = nullptr);

    void setTelemetry(const JointTelemetry &telem);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QString m_axis_name;
    JointTelemetry m_telem;
};

/// Dashboard showing all axes in a compact grid format with quick status
class AxisOverviewPanel : public QWidget
{
    Q_OBJECT

public:
    explicit AxisOverviewPanel(QWidget *parent = nullptr);

    void setJoints(const std::vector<JointInfo> &joints);
    void updateTelemetry(const std::vector<JointTelemetry> &joints);

private:
    std::vector<AxisStatusWidget *> m_status_widgets;
    std::vector<JointInfo> m_joints;
};

} // namespace actuator_test::gui
