// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/trajectory_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QVBoxLayout>
#include <QVariant>

namespace actuator_test::gui
{

TrajectoryPanel::TrajectoryPanel(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *form = new QFormLayout();
    m_mode_combo = new QComboBox();
    m_mode_combo->addItem(tr("Sinusoid"), QVariant::fromValue(static_cast<int>(TrajectoryMode::Sin)));
    m_mode_combo->addItem(tr("Chirp (linear sweep)"),
                          QVariant::fromValue(static_cast<int>(TrajectoryMode::ChirpLinear)));
    m_mode_combo->addItem(tr("Chirp (log sweep)"), QVariant::fromValue(static_cast<int>(TrajectoryMode::ChirpLog)));
    m_mode_combo->addItem(tr("Triangle"), QVariant::fromValue(static_cast<int>(TrajectoryMode::Triangle)));
    m_mode_combo->addItem(tr("Step (square)"), QVariant::fromValue(static_cast<int>(TrajectoryMode::Step)));
    m_mode_combo->addItem(tr("Multisine"), QVariant::fromValue(static_cast<int>(TrajectoryMode::Multisine)));
    m_mode_combo->addItem(tr("Spline (recorded)"), QVariant::fromValue(static_cast<int>(TrajectoryMode::Spline)));
    form->addRow(tr("Waveform:"), m_mode_combo);
    layout->addLayout(form);

    m_hint = new QLabel(tr("Parametric waveforms run within the captured limits."));
    m_hint->setWordWrap(true);
    layout->addWidget(m_hint);

    m_record_btn = new QPushButton(tr("Record spline path"));
    m_record_btn->setCheckable(true);
    m_record_btn->setVisible(false);
    layout->addWidget(m_record_btn);

    m_log_check = new QCheckBox(tr("Record CSV log"));
    m_log_check->setChecked(true);
    layout->addWidget(m_log_check);

    auto *btn_row = new QHBoxLayout();
    m_play_btn = new QPushButton(tr("Play"));
    m_pause_btn = new QPushButton(tr("Pause"));
    m_pause_btn->setEnabled(false);
    m_stop_btn = new QPushButton(tr("Stop"));
    m_stop_btn->setEnabled(false);
    btn_row->addWidget(m_play_btn);
    btn_row->addWidget(m_pause_btn);
    btn_row->addWidget(m_stop_btn);
    layout->addLayout(btn_row);
    layout->addStretch(1);

    connect(m_record_btn, &QPushButton::toggled, this, [this](bool on) {
        m_record_btn->setText(on ? tr("Finish recording") : tr("Record spline path"));
        emit splineRecordToggled(on);
    });
    connect(m_play_btn, &QPushButton::clicked, this,
            [this] { emit playRequested(selectedMode(), loggingEnabled()); });
    connect(m_pause_btn, &QPushButton::clicked, this, &TrajectoryPanel::pauseRequested);
    connect(m_stop_btn, &QPushButton::clicked, this, &TrajectoryPanel::stopRequested);

    connect(m_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int) {
        if (selectedMode() == TrajectoryMode::Spline)
        {
            m_record_btn->setVisible(true);
            m_hint->setText(
                tr("1) Press 'Record spline path' and backdrive the joint. 2) Finish recording. 3) Press Play."));
        }
        else
        {
            m_record_btn->setVisible(false);
            m_hint->setText(tr("Parametric waveforms run within the captured limits."));
        }
    });

    setEnabled(false);
}

void TrajectoryPanel::setRunning(bool running)
{
    m_play_btn->setEnabled(!running);
    m_pause_btn->setEnabled(running);
    m_stop_btn->setEnabled(running);
    m_mode_combo->setEnabled(!running);
    m_log_check->setEnabled(!running);
    m_record_btn->setEnabled(!running);

    if (running)
    {
        m_hint->setText(tr("Trajectory running. Use Pause to hold position or Stop to idle all drives."));
    }
    else if (selectedMode() == TrajectoryMode::Spline)
    {
        m_hint->setText(
            tr("1) Press 'Record spline path' and backdrive the joint. 2) Finish recording. 3) Press Play."));
    }
    else
    {
        m_hint->setText(tr("Parametric waveforms run within the captured limits."));
    }
}

void TrajectoryPanel::setCapturing(bool capturing)
{
    if (!m_record_btn->isVisible())
    {
        return;
    }
    if (m_record_btn->isChecked() == capturing)
    {
        return;
    }
    m_record_btn->blockSignals(true);
    m_record_btn->setChecked(capturing);
    m_record_btn->setText(capturing ? tr("Finish recording") : tr("Record spline path"));
    m_record_btn->blockSignals(false);

    if (capturing && selectedMode() == TrajectoryMode::Spline)
    {
        m_hint->setText(tr("Recording spline path. Backdrive now, then click 'Finish recording'."));
    }
}

void TrajectoryPanel::setSelectedMode(TrajectoryMode mode)
{
    const int idx = m_mode_combo->findData(QVariant::fromValue(static_cast<int>(mode)));
    if (idx >= 0)
    {
        m_mode_combo->setCurrentIndex(idx);
    }
}

void TrajectoryPanel::setLoggingEnabled(bool enabled)
{
    m_log_check->setChecked(enabled);
}

TrajectoryMode TrajectoryPanel::selectedMode() const
{
    return static_cast<TrajectoryMode>(m_mode_combo->currentData().toInt());
}

bool TrajectoryPanel::loggingEnabled() const
{
    return m_log_check->isChecked();
}

} // namespace actuator_test::gui
