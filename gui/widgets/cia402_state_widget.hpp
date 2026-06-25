// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// CiA-402 state machine visualization widget showing drive operational state.

#pragma once

#include "core/telemetry.hpp"

#include <QWidget>

namespace actuator_test::gui
{

/// Displays the CiA-402 state machine as a professional industrial diagram
/// with state transitions and current state highlighted.
class CiA402StateWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CiA402StateWidget(QWidget *parent = nullptr);

    /// Update the displayed state and fault status.
    void setState(CiA402State state, bool fault);

    /// Get current displayed state.
    CiA402State state() const { return m_current_state; }

protected:
    void paintEvent(QPaintEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;

private:
    struct StateBox
    {
        QString name;
        int x = 0;
        int y = 0;
        int w = 0;
        int h = 0;
        CiA402State state = CiA402State::Unknown;
    };

    void layoutStates();
    StateBox stateAt(CiA402State state) const;
    QColor stateColor(CiA402State state, bool active) const;

    CiA402State m_current_state = CiA402State::NotReadyToSwitchOn;
    bool m_fault = false;
    std::vector<StateBox> m_state_boxes;
};

} // namespace actuator_test::gui
