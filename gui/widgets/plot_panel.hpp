// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Live plotting of reference vs. actual position and tracking error for the
// joint currently selected in the combo box.  Implemented as a lightweight
// custom-painted strip chart so the GUI depends only on Qt Widgets (no extra
// charting module).

#pragma once

#include "core/telemetry.hpp"

#include <QColor>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QWidget>
#include <deque>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
QT_END_NAMESPACE

namespace actuator_test::gui
{

/// A minimal rolling-window multi-series strip chart.  Not a QObject: it has no
/// signals/slots, just custom painting.
class StripChart : public QWidget
{
public:
    explicit StripChart(QString title, QWidget *parent = nullptr);

    /// Register a named series and return its index.
    int addSeries(const QString &name, const QColor &color);

    /// Append a sample to a series (x is seconds).
    void append(int series, double x, double y);

    /// Force the vertical axis to be symmetric about zero (used for error).
    void setSymmetric(bool symmetric);

    void setWindowSeconds(double seconds);
    void clearAll();

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    struct Series
    {
        QString name;
        QColor color;
        std::deque<QPointF> points;
    };

    QString m_title;
    std::vector<Series> m_series;
    double m_window_s = 10.0;
    bool m_symmetric = false;
    double m_latest_x = 0.0;
};

/// The dockable plot panel: a joint selector plus tracking, error, and
/// velocity charts.
class PlotPanel : public QWidget
{
    Q_OBJECT

public:
    explicit PlotPanel(QWidget *parent = nullptr);

    void setJoints(const QStringList &names);
    void appendFrame(const TelemetryFrame &frame);
    void clearHistory();

    /// Set the update rate for plot refresh (milliseconds between updates).
    /// Lower values = more responsive but higher CPU. Default 50 ms.
    void setUpdateRate(uint32_t ms) { m_update_rate_ms = ms; }

    /// Set the time window displayed by the charts (seconds). Default 10s.
    void setWindowSeconds(double seconds);

    /// Enable/disable specific plot types.
    void setShowVelocity(bool show);
    void setShowFollowingError(bool show);

private slots:
    void onJointChanged(int index);

private:
    QComboBox *m_joint_combo = nullptr;
    StripChart *m_track = nullptr;
    StripChart *m_error = nullptr;
    StripChart *m_velocity = nullptr;
    StripChart *m_following_error = nullptr;

    int m_ref_series = 0;
    int m_act_series = 0;
    int m_err_series = 0;
    int m_vel_ref_series = 0;
    int m_vel_act_series = 0;
    int m_foll_err_series = 0;

    int m_selected = 0;
    uint32_t m_update_rate_ms = 50;
    uint32_t m_elapsed_ms = 0;
};

} // namespace actuator_test::gui
