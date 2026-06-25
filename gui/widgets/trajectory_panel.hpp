// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Pick a characterisation waveform and play it across the ticked joints.

#pragma once

#include "actuator_test/types.hpp"

#include <QWidget>

QT_BEGIN_NAMESPACE
class QComboBox;
class QCheckBox;
class QLabel;
class QPushButton;
QT_END_NAMESPACE

namespace actuator_test::gui
{

class TrajectoryPanel : public QWidget
{
    Q_OBJECT

public:
    explicit TrajectoryPanel(QWidget *parent = nullptr);

    void setRunning(bool running);
    void setCapturing(bool capturing);
    void setSelectedMode(TrajectoryMode mode);
    void setLoggingEnabled(bool enabled);

    TrajectoryMode selectedMode() const;
    bool loggingEnabled() const;

signals:
    void splineRecordToggled(bool start);
    void playRequested(TrajectoryMode mode, bool enable_logging);
    void pauseRequested();
    void stopRequested();

private:
    QComboBox *m_mode_combo = nullptr;
    QCheckBox *m_log_check = nullptr;
    QLabel *m_hint = nullptr;
    QPushButton *m_record_btn = nullptr;
    QPushButton *m_play_btn = nullptr;
    QPushButton *m_pause_btn = nullptr;
    QPushButton *m_stop_btn = nullptr;
};

} // namespace actuator_test::gui
