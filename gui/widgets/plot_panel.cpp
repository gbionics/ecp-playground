// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/plot_panel.hpp"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPaintEvent>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace actuator_test::gui
{

// ---------------------------------------------------------------------------
//  StripChart
// ---------------------------------------------------------------------------

StripChart::StripChart(QString title, QWidget *parent) : QWidget(parent), m_title(std::move(title))
{
    setMinimumHeight(140);
    setAutoFillBackground(true);
}

int StripChart::addSeries(const QString &name, const QColor &color)
{
    m_series.push_back({name, color, {}});
    return static_cast<int>(m_series.size()) - 1;
}

void StripChart::setSymmetric(bool symmetric)
{
    m_symmetric = symmetric;
}

void StripChart::setWindowSeconds(double seconds)
{
    m_window_s = seconds;
}

void StripChart::clearAll()
{
    for (auto &s : m_series)
    {
        s.points.clear();
    }
    m_latest_x = 0.0;
    update();
}

void StripChart::append(int series, double x, double y)
{
    if (series < 0 || series >= static_cast<int>(m_series.size()))
    {
        return;
    }
    auto &pts = m_series[static_cast<std::size_t>(series)].points;
    pts.emplace_back(x, y);
    m_latest_x = std::max(m_latest_x, x);
    const double x_min = m_latest_x - m_window_s;
    while (pts.size() > 1 && pts.front().x() < x_min)
    {
        pts.pop_front();
    }
    update();
}

void StripChart::paintEvent(QPaintEvent *)
{
    QPainter p(this);
    p.fillRect(rect(), QColor(26, 28, 32));

    const int left = 48;
    const int right = 10;
    const int top = 20;
    const int bottom = 18;
    const QRectF plot(left, top, std::max(1, width() - left - right), std::max(1, height() - top - bottom));

    p.setPen(QColor(200, 200, 200));
    p.drawText(QRectF(0, 0, width(), top), Qt::AlignCenter, m_title);

    // X window.
    const double x_max = std::max(m_latest_x, m_window_s);
    const double x_min = x_max - m_window_s;

    // Y range across all visible points.
    double y_lo = 0.0, y_hi = 0.0;
    bool have = false;
    for (const auto &s : m_series)
    {
        for (const auto &pt : s.points)
        {
            if (!have)
            {
                y_lo = y_hi = pt.y();
                have = true;
            }
            else
            {
                y_lo = std::min(y_lo, pt.y());
                y_hi = std::max(y_hi, pt.y());
            }
        }
    }
    if (!have)
    {
        y_lo = -1.0;
        y_hi = 1.0;
    }
    if (m_symmetric)
    {
        const double a = std::max({std::abs(y_lo), std::abs(y_hi), 0.5});
        y_lo = -a;
        y_hi = a;
    }
    const double pad = std::max(0.5, (y_hi - y_lo) * 0.1);
    y_lo -= pad;
    y_hi += pad;
    const double y_span = std::max(1e-6, y_hi - y_lo);
    const double x_span = std::max(1e-6, x_max - x_min);

    auto toPx = [&](const QPointF &pt) {
        const double fx = (pt.x() - x_min) / x_span;
        const double fy = (pt.y() - y_lo) / y_span;
        return QPointF(plot.left() + fx * plot.width(), plot.bottom() - fy * plot.height());
    };

    // Grid + axis ticks.
    const QColor grid_col(48, 52, 58);
    const QColor label_col(150, 154, 160);
    const QFont f = p.font();
    p.setFont(f);

    const int y_divs = 4;
    for (int i = 0; i <= y_divs; ++i)
    {
        const double frac = static_cast<double>(i) / y_divs;
        const double y = plot.bottom() - frac * plot.height();
        const double value = y_lo + frac * y_span;
        p.setPen(grid_col);
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
        p.setPen(label_col);
        p.drawText(QRectF(0, y - 8, plot.left() - 4, 16), Qt::AlignRight | Qt::AlignVCenter,
                   QString::number(value, 'f', 1));
    }

    const int x_divs = 5;
    for (int i = 0; i <= x_divs; ++i)
    {
        const double frac = static_cast<double>(i) / x_divs;
        const double x = plot.left() + frac * plot.width();
        const double value = x_min + frac * x_span;
        p.setPen(grid_col);
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
        p.setPen(label_col);
        p.drawText(QRectF(x - 24, plot.bottom() + 2, 48, bottom - 2), Qt::AlignCenter, QString::number(value, 'f', 1));
    }

    // Plot border + zero line.
    p.setPen(QColor(80, 84, 90));
    p.drawRect(plot);
    const double zero_y = plot.bottom() - ((0.0 - y_lo) / y_span) * plot.height();
    if (zero_y >= plot.top() && zero_y <= plot.bottom())
    {
        p.setPen(QColor(110, 114, 120));
        p.drawLine(QPointF(plot.left(), zero_y), QPointF(plot.right(), zero_y));
    }

    // Series.
    int legend_x = static_cast<int>(plot.left()) + 6;
    for (const auto &s : m_series)
    {
        if (s.points.size() >= 2)
        {
            QPolygonF poly;
            poly.reserve(static_cast<int>(s.points.size()));
            for (const auto &pt : s.points)
            {
                poly << toPx(pt);
            }
            p.setPen(QPen(s.color, 1.5));
            p.drawPolyline(poly);
        }
        // Legend swatch.
        p.setPen(s.color);
        p.drawText(QPointF(legend_x, plot.top() + 12), s.name);
        legend_x += p.fontMetrics().horizontalAdvance(s.name) + 18;
    }
}

// ---------------------------------------------------------------------------
//  PlotPanel
// ---------------------------------------------------------------------------

PlotPanel::PlotPanel(QWidget *parent) : QWidget(parent)
{
    auto *layout = new QVBoxLayout(this);

    auto *top = new QHBoxLayout();
    top->addWidget(new QLabel(tr("Joint:")));
    m_joint_combo = new QComboBox();
    top->addWidget(m_joint_combo, 1);
    layout->addLayout(top);

    m_track = new StripChart(tr("Position tracking [deg]"));
    m_ref_series = m_track->addSeries(tr("reference"), QColor(80, 160, 255));
    m_act_series = m_track->addSeries(tr("actual"), QColor(90, 220, 120));
    layout->addWidget(m_track, 3);

    m_error = new StripChart(tr("Tracking error [deg]"));
    m_error->setSymmetric(true);
    m_err_series = m_error->addSeries(tr("error"), QColor(240, 120, 110));
    layout->addWidget(m_error, 2);

    m_velocity = new StripChart(tr("Velocity [deg/s]"));
    m_vel_ref_series = m_velocity->addSeries(tr("reference"), QColor(100, 180, 255));
    m_vel_act_series = m_velocity->addSeries(tr("actual"), QColor(110, 240, 140));
    layout->addWidget(m_velocity, 2);

    m_following_error = new StripChart(tr("Following Error - Filtered [deg]"));
    m_following_error->setSymmetric(true);
    m_foll_err_series = m_following_error->addSeries(tr("following error"), QColor(220, 80, 160));
    layout->addWidget(m_following_error, 1);

    connect(m_joint_combo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &PlotPanel::onJointChanged);
}

void PlotPanel::setJoints(const QStringList &names)
{
    const QString current = m_joint_combo->currentText();
    m_joint_combo->blockSignals(true);
    m_joint_combo->clear();
    m_joint_combo->addItems(names);
    const int idx = names.indexOf(current);
    m_selected = (idx >= 0) ? idx : 0;
    m_joint_combo->setCurrentIndex(m_selected);
    m_joint_combo->blockSignals(false);
    clearHistory();
}

void PlotPanel::onJointChanged(int index)
{
    m_selected = std::max(0, index);
    clearHistory();
}

void PlotPanel::clearHistory()
{
    m_track->clearAll();
    m_error->clearAll();
    m_velocity->clearAll();
    m_following_error->clearAll();
    m_elapsed_ms = 0;
}

void PlotPanel::setWindowSeconds(double seconds)
{
    m_track->setWindowSeconds(seconds);
    m_error->setWindowSeconds(seconds);
    m_velocity->setWindowSeconds(seconds);
    m_following_error->setWindowSeconds(seconds);
}

void PlotPanel::setShowVelocity(bool show)
{
    m_velocity->setVisible(show);
}

void PlotPanel::setShowFollowingError(bool show)
{
    m_following_error->setVisible(show);
}

void PlotPanel::appendFrame(const TelemetryFrame &frame)
{
    if (m_selected < 0 || static_cast<std::size_t>(m_selected) >= frame.joints.size())
    {
        return;
    }

    // Accumulate time, update plots only when threshold reached (throttling).
    static uint32_t last_update_time = 0;
    const auto now_ms = static_cast<uint32_t>(frame.t_s * 1000.0);
    const uint32_t dt_ms = (now_ms > last_update_time) ? (now_ms - last_update_time) : 16;
    m_elapsed_ms += dt_ms;

    if (m_elapsed_ms < m_update_rate_ms)
    {
        return; // Skip this update; buffer for next batch
    }

    last_update_time = now_ms;
    m_elapsed_ms = 0;

    const JointTelemetry &j = frame.joints[static_cast<std::size_t>(m_selected)];

    // Position and error
    m_track->append(m_ref_series, frame.t_s, j.reference_deg);
    m_track->append(m_act_series, frame.t_s, j.actual_deg);
    m_error->append(m_err_series, frame.t_s, j.error_deg);

    // Velocity
    m_velocity->append(m_vel_ref_series, frame.t_s, j.ref_velocity_deg_s);
    m_velocity->append(m_vel_act_series, frame.t_s, j.velocity_deg_s);

    // Following error
    m_following_error->append(m_foll_err_series, frame.t_s, j.following_error_deg);
}

} // namespace actuator_test::gui
