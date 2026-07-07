// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/plot_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QPaintEvent>
#include <QPainter>
#include <QPalette>
#include <QPushButton>
#include <QScrollArea>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>

namespace actuator_test::gui {

// ---------------------------------------------------------------------------
//  StripChart
// ---------------------------------------------------------------------------

StripChart::StripChart(QString title, QWidget *parent)
    : QWidget(parent), m_title(std::move(title)) {
  setMinimumHeight(140);
  setAutoFillBackground(true);
}

int StripChart::addSeries(const QString &name, const QColor &color) {
  m_series.push_back({name, color, {}});
  return static_cast<int>(m_series.size()) - 1;
}

void StripChart::resetSeries() {
  m_series.clear();
  m_latest_x = 0.0;
}

void StripChart::setSymmetric(bool symmetric) { m_symmetric = symmetric; }

void StripChart::setWindowSeconds(double seconds) { m_window_s = seconds; }

void StripChart::setAxisTitles(const QString &x, const QString &y) {
  m_x_title = x;
  m_y_title = y;
}

void StripChart::clearAll() {
  for (auto &s : m_series) {
    s.points.clear();
  }
  m_latest_x = 0.0;
  update();
}

void StripChart::append(int series, double x, double y) {
  if (series < 0 || series >= static_cast<int>(m_series.size())) {
    return;
  }
  auto &pts = m_series[static_cast<std::size_t>(series)].points;
  pts.emplace_back(x, y);

  if (m_x_axis == XAxis::Time) {
    m_latest_x = std::max(m_latest_x, x);
    const double x_min = m_latest_x - m_window_s;
    while (pts.size() > 1 && pts.front().x() < x_min) {
      pts.pop_front();
    }
  } else {
    // Value (XY) axis: cap by sample count so the trace does not grow
    // without bound.
    while (pts.size() > m_max_points) {
      pts.pop_front();
    }
  }
  update();
}

void StripChart::paintEvent(QPaintEvent *) {
  QPainter p(this);
  p.fillRect(rect(), QColor(26, 28, 32));

  const int left = 52;
  const int right = 10;
  const int top = 20;
  const int bottom = m_x_title.isEmpty() ? 18 : 30;
  const QRectF plot(left, top, std::max(1, width() - left - right),
                    std::max(1, height() - top - bottom));

  p.setPen(QColor(200, 200, 200));
  p.drawText(QRectF(0, 0, width(), top), Qt::AlignCenter, m_title);

  // Determine axis ranges.
  double x_min = 0.0, x_max = m_window_s;
  if (m_x_axis == XAxis::Value) {
    bool have_x = false;
    for (const auto &s : m_series) {
      for (const auto &pt : s.points) {
        if (!have_x) {
          x_min = x_max = pt.x();
          have_x = true;
        } else {
          x_min = std::min(x_min, pt.x());
          x_max = std::max(x_max, pt.x());
        }
      }
    }
    if (!have_x) {
      x_min = -1.0;
      x_max = 1.0;
    }
    const double xp = std::max(0.5, (x_max - x_min) * 0.08);
    x_min -= xp;
    x_max += xp;
  } else {
    x_max = std::max(m_latest_x, m_window_s);
    x_min = x_max - m_window_s;
  }

  // Y range across all visible points.
  double y_lo = 0.0, y_hi = 0.0;
  bool have = false;
  for (const auto &s : m_series) {
    for (const auto &pt : s.points) {
      if (!have) {
        y_lo = y_hi = pt.y();
        have = true;
      } else {
        y_lo = std::min(y_lo, pt.y());
        y_hi = std::max(y_hi, pt.y());
      }
    }
  }
  if (!have) {
    y_lo = -1.0;
    y_hi = 1.0;
  }
  if (m_symmetric) {
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
    return QPointF(plot.left() + fx * plot.width(),
                   plot.bottom() - fy * plot.height());
  };

  // Grid + axis ticks.
  const QColor grid_col(48, 52, 58);
  const QColor label_col(150, 154, 160);

  const int y_divs = 4;
  for (int i = 0; i <= y_divs; ++i) {
    const double frac = static_cast<double>(i) / y_divs;
    const double y = plot.bottom() - frac * plot.height();
    const double value = y_lo + frac * y_span;
    p.setPen(grid_col);
    p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    p.setPen(label_col);
    p.drawText(QRectF(0, y - 8, plot.left() - 4, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(value, 'f', 1));
  }

  const int x_divs = 5;
  for (int i = 0; i <= x_divs; ++i) {
    const double frac = static_cast<double>(i) / x_divs;
    const double x = plot.left() + frac * plot.width();
    const double value = x_min + frac * x_span;
    p.setPen(grid_col);
    p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    p.setPen(label_col);
    p.drawText(QRectF(x - 24, plot.bottom() + 2, 48, 14), Qt::AlignCenter,
               QString::number(value, 'f', 1));
  }

  if (!m_x_title.isEmpty()) {
    p.setPen(label_col);
    p.drawText(QRectF(plot.left(), plot.bottom() + 14, plot.width(), 14),
               Qt::AlignCenter, m_x_title);
  }

  // Plot border + zero line.
  p.setPen(QColor(80, 84, 90));
  p.drawRect(plot);
  const double zero_y = plot.bottom() - ((0.0 - y_lo) / y_span) * plot.height();
  if (zero_y >= plot.top() && zero_y <= plot.bottom()) {
    p.setPen(QColor(110, 114, 120));
    p.drawLine(QPointF(plot.left(), zero_y), QPointF(plot.right(), zero_y));
  }

  // Series.
  int legend_x = static_cast<int>(plot.left()) + 6;
  for (const auto &s : m_series) {
    if (s.points.size() >= 2) {
      QPolygonF poly;
      poly.reserve(static_cast<int>(s.points.size()));
      for (const auto &pt : s.points) {
        poly << toPx(pt);
      }
      p.setPen(QPen(s.color, 1.5));
      p.drawPolyline(poly);
    } else if (s.points.size() == 1) {
      p.setPen(QPen(s.color, 3.0));
      p.drawPoint(toPx(s.points.front()));
    }
    // Legend swatch.
    if (!s.name.isEmpty()) {
      p.setPen(s.color);
      p.drawText(QPointF(legend_x, plot.top() + 12), s.name);
      legend_x += p.fontMetrics().horizontalAdvance(s.name) + 18;
    }
  }
}

// ---------------------------------------------------------------------------
//  PlotPanel
// ---------------------------------------------------------------------------

PlotPanel::PlotPanel(QWidget *parent) : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);
  layout->setContentsMargins(6, 6, 6, 6);
  layout->setSpacing(6);

  // --- Controls row ------------------------------------------------------
  auto *top = new QHBoxLayout();

  auto *mode_label = new QLabel(tr("&View:"));
  m_mode_combo = new QComboBox();
  m_mode_combo->addItem(tr("Focus (one joint)"),
                        static_cast<int>(ViewMode::Focus));
  m_mode_combo->addItem(tr("Overlay joints"),
                        static_cast<int>(ViewMode::Overlay));
  m_mode_combo->addItem(tr("Stacked per joint"),
                        static_cast<int>(ViewMode::Stacked));
  m_mode_combo->addItem(tr("XY phase plot"), static_cast<int>(ViewMode::XY));
  m_mode_combo->setAccessibleName(tr("Plot view mode"));
  m_mode_combo->setToolTip(
      tr("Focus: one joint in detail. Overlay: one signal across joints. "
         "Stacked: a chart per joint. XY: plot one signal against another."));
  mode_label->setBuddy(m_mode_combo);
  top->addWidget(mode_label);
  top->addWidget(m_mode_combo);

  auto add_signal_combo = [this](QComboBox *&combo, Signal def) {
    combo = new QComboBox();
    combo->addItem(signalName(Signal::PositionActual),
                   static_cast<int>(Signal::PositionActual));
    combo->addItem(signalName(Signal::PositionRef),
                   static_cast<int>(Signal::PositionRef));
    combo->addItem(signalName(Signal::Error), static_cast<int>(Signal::Error));
    combo->addItem(signalName(Signal::Velocity),
                   static_cast<int>(Signal::Velocity));
    combo->addItem(signalName(Signal::RefVelocity),
                   static_cast<int>(Signal::RefVelocity));
    combo->addItem(signalName(Signal::FollowingError),
                   static_cast<int>(Signal::FollowingError));
    combo->setCurrentIndex(combo->findData(static_cast<int>(def)));
  };

  m_signal_label = new QLabel(tr("&Signal:"));
  add_signal_combo(m_signal_combo, Signal::PositionActual);
  m_signal_combo->setAccessibleName(tr("Overlay signal"));
  m_signal_label->setBuddy(m_signal_combo);
  top->addWidget(m_signal_label);
  top->addWidget(m_signal_combo);

  m_x_signal_label = new QLabel(tr("&X:"));
  add_signal_combo(m_x_signal_combo, Signal::PositionActual);
  m_x_signal_combo->setAccessibleName(tr("XY plot X signal"));
  m_x_signal_label->setBuddy(m_x_signal_combo);
  top->addWidget(m_x_signal_label);
  top->addWidget(m_x_signal_combo);

  m_y_signal_label = new QLabel(tr("&Y:"));
  add_signal_combo(m_y_signal_combo, Signal::Velocity);
  m_y_signal_combo->setAccessibleName(tr("XY plot Y signal"));
  m_y_signal_label->setBuddy(m_y_signal_combo);
  top->addWidget(m_y_signal_label);
  top->addWidget(m_y_signal_combo);

  top->addStretch(1);

  top->addWidget(new QLabel(tr("Window:")));
  m_window_combo = new QComboBox();
  m_window_combo->setAccessibleName(tr("Plot time window"));
  m_window_combo->setToolTip(tr("How many seconds of history to display."));
  m_window_combo->addItem(tr("5 s"), 5.0);
  m_window_combo->addItem(tr("10 s"), 10.0);
  m_window_combo->addItem(tr("20 s"), 20.0);
  m_window_combo->addItem(tr("60 s"), 60.0);
  m_window_combo->setCurrentIndex(1);
  top->addWidget(m_window_combo);

  m_freeze_btn = new QPushButton(tr("&Freeze"));
  m_freeze_btn->setCheckable(true);
  m_freeze_btn->setAccessibleName(tr("Freeze plots"));
  m_freeze_btn->setToolTip(
      tr("Pause live plotting to inspect the current window."));
  top->addWidget(m_freeze_btn);

  auto *clear_btn = new QPushButton(tr("&Clear"));
  clear_btn->setAccessibleName(tr("Clear plots"));
  clear_btn->setToolTip(tr("Clear all plotted history."));
  top->addWidget(clear_btn);

  layout->addLayout(top);

  // --- Joint selection row (checkboxes) ---------------------------------
  auto *joints_row = new QHBoxLayout();
  joints_row->setContentsMargins(0, 0, 0, 0);
  joints_row->addWidget(new QLabel(tr("Joints:")));
  m_joint_checks_host = new QWidget();
  auto *checks_layout = new QHBoxLayout(m_joint_checks_host);
  checks_layout->setContentsMargins(0, 0, 0, 0);
  checks_layout->setSpacing(10);
  auto *checks_scroll = new QScrollArea();
  checks_scroll->setWidgetResizable(true);
  checks_scroll->setFrameShape(QFrame::NoFrame);
  checks_scroll->setWidget(m_joint_checks_host);
  checks_scroll->setMaximumHeight(38);
  checks_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  checks_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
  joints_row->addWidget(checks_scroll, 1);
  layout->addLayout(joints_row);

  // --- Charts container --------------------------------------------------
  m_charts_container = new QWidget();
  m_charts_layout = new QVBoxLayout(m_charts_container);
  m_charts_layout->setContentsMargins(0, 0, 0, 0);
  m_charts_layout->setSpacing(6);
  layout->addWidget(m_charts_container, 1);

  // --- Wiring ------------------------------------------------------------
  connect(m_mode_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) {
            m_mode = static_cast<ViewMode>(m_mode_combo->currentData().toInt());
            rebuildCharts();
          });
  connect(m_signal_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { rebuildCharts(); });
  connect(m_x_signal_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { rebuildCharts(); });
  connect(m_y_signal_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) { rebuildCharts(); });
  connect(m_window_combo, QOverload<int>::of(&QComboBox::currentIndexChanged),
          this, [this](int) {
            setWindowSeconds(m_window_combo->currentData().toDouble());
          });
  connect(m_freeze_btn, &QPushButton::toggled, this, [this](bool on) {
    m_frozen = on;
    m_freeze_btn->setText(on ? tr("&Resume") : tr("&Freeze"));
  });
  connect(clear_btn, &QPushButton::clicked, this, &PlotPanel::clearHistory);

  m_window_s = 10.0;
  rebuildCharts();
}

QColor PlotPanel::jointColor(int index) {
  static const QColor palette[] = {
      QColor(90, 220, 120),  QColor(80, 160, 255),  QColor(240, 120, 110),
      QColor(230, 200, 90),  QColor(200, 120, 240), QColor(90, 220, 220),
      QColor(240, 150, 80),  QColor(160, 200, 120), QColor(220, 100, 170),
      QColor(120, 170, 240),
  };
  const int n = static_cast<int>(sizeof(palette) / sizeof(palette[0]));
  return palette[((index % n) + n) % n];
}

QString PlotPanel::signalName(Signal s) {
  switch (s) {
  case Signal::PositionActual:
    return tr("Position [deg]");
  case Signal::PositionRef:
    return tr("Reference [deg]");
  case Signal::Error:
    return tr("Error [deg]");
  case Signal::Velocity:
    return tr("Velocity [deg/s]");
  case Signal::RefVelocity:
    return tr("Ref velocity [deg/s]");
  case Signal::FollowingError:
    return tr("Following error [deg]");
  }
  return {};
}

bool PlotPanel::signalIsSymmetric(Signal s) {
  return s == Signal::Error || s == Signal::FollowingError;
}

double PlotPanel::signalValue(const JointTelemetry &j, Signal s) {
  switch (s) {
  case Signal::PositionActual:
    return j.actual_deg;
  case Signal::PositionRef:
    return j.reference_deg;
  case Signal::Error:
    return j.error_deg;
  case Signal::Velocity:
    return j.velocity_deg_s;
  case Signal::RefVelocity:
    return j.ref_velocity_deg_s;
  case Signal::FollowingError:
    return j.following_error_deg;
  }
  return 0.0;
}

PlotPanel::Signal PlotPanel::signalAt(const QComboBox *combo) const {
  if (combo == nullptr) {
    return Signal::PositionActual;
  }
  return static_cast<Signal>(combo->currentData().toInt());
}

std::vector<int> PlotPanel::checkedJoints() const {
  std::vector<int> out;
  for (int i = 0; i < m_joint_checks.size(); ++i) {
    if (m_joint_checks[i]->isChecked()) {
      out.push_back(i);
    }
  }
  return out;
}

void PlotPanel::setJoints(const QStringList &names) {
  m_joint_names = names;

  // Rebuild the joint checkboxes.
  for (auto *cb : m_joint_checks) {
    cb->deleteLater();
  }
  m_joint_checks.clear();

  auto *host_layout =
      qobject_cast<QHBoxLayout *>(m_joint_checks_host->layout());
  if (host_layout != nullptr) {
    // Remove any leftover stretch items.
    QLayoutItem *item = nullptr;
    while ((item = host_layout->takeAt(0)) != nullptr) {
      delete item;
    }
  }

  for (int i = 0; i < names.size(); ++i) {
    auto *cb = new QCheckBox(names[i], m_joint_checks_host);
    cb->setChecked(i == 0); // focus the first joint by default
    cb->setAccessibleName(tr("Plot joint %1").arg(names[i]));
    QPalette pal = cb->palette();
    pal.setColor(QPalette::WindowText, jointColor(i));
    cb->setPalette(pal);
    connect(cb, &QCheckBox::toggled, this, [this](bool) { rebuildCharts(); });
    if (host_layout != nullptr) {
      host_layout->addWidget(cb);
    }
    m_joint_checks.push_back(cb);
  }
  if (host_layout != nullptr) {
    host_layout->addStretch(1);
  }

  rebuildCharts();
}

void PlotPanel::rebuildCharts() {
  if (m_charts_layout == nullptr) {
    return;
  }

  // Tear down previous charts.
  m_items.clear();
  m_charts.clear();
  QLayoutItem *item = nullptr;
  while ((item = m_charts_layout->takeAt(0)) != nullptr) {
    if (QWidget *w = item->widget()) {
      w->deleteLater();
    }
    delete item;
  }

  // Toggle which controls are relevant for the active mode.
  const bool overlay = (m_mode == ViewMode::Overlay);
  const bool xy = (m_mode == ViewMode::XY);
  m_signal_label->setVisible(overlay);
  m_signal_combo->setVisible(overlay);
  m_x_signal_label->setVisible(xy);
  m_x_signal_combo->setVisible(xy);
  m_y_signal_label->setVisible(xy);
  m_y_signal_combo->setVisible(xy);
  m_window_combo->setEnabled(!xy);

  std::vector<int> joints = checkedJoints();
  if (joints.empty() && !m_joint_names.isEmpty()) {
    // Always keep at least one joint plotted.
    if (!m_joint_checks.isEmpty()) {
      const QSignalBlocker blocker(m_joint_checks[0]);
      m_joint_checks[0]->setChecked(true);
    }
    joints.push_back(0);
  }

  auto add_chart = [this](const QString &title, bool symmetric, int stretch) {
    auto *c = new StripChart(title);
    c->setSymmetric(symmetric);
    c->setWindowSeconds(m_window_s);
    m_charts.push_back(c);
    m_charts_layout->addWidget(c, stretch);
    return static_cast<int>(m_charts.size()) - 1;
  };

  switch (m_mode) {
  case ViewMode::Focus: {
    const int j = joints.empty() ? -1 : joints.front();
    const QString jn =
        (j >= 0 && j < m_joint_names.size()) ? m_joint_names[j] : QString();

    const int c_pos =
        add_chart(tr("Position tracking [deg] — %1").arg(jn), false, 3);
    const int s_ref =
        m_charts[c_pos]->addSeries(tr("reference"), QColor(80, 160, 255));
    const int s_act =
        m_charts[c_pos]->addSeries(tr("actual"), QColor(90, 220, 120));

    const int c_err = add_chart(tr("Tracking error [deg]"), true, 2);
    const int s_err =
        m_charts[c_err]->addSeries(tr("error"), QColor(240, 120, 110));

    const int c_vel = add_chart(tr("Velocity [deg/s]"), false, 2);
    const int s_vref =
        m_charts[c_vel]->addSeries(tr("reference"), QColor(100, 180, 255));
    const int s_vact =
        m_charts[c_vel]->addSeries(tr("actual"), QColor(110, 240, 140));

    const int c_foll = add_chart(tr("Following error [deg]"), true, 1);
    const int s_foll = m_charts[c_foll]->addSeries(tr("following error"),
                                                   QColor(220, 80, 160));

    if (j >= 0) {
      m_items.push_back({c_pos, s_ref, j, Signal::PositionRef,
                         Signal::PositionActual, false});
      m_items.push_back({c_pos, s_act, j, Signal::PositionActual,
                         Signal::PositionActual, false});
      m_items.push_back(
          {c_err, s_err, j, Signal::Error, Signal::PositionActual, false});
      m_items.push_back({c_vel, s_vref, j, Signal::RefVelocity,
                         Signal::PositionActual, false});
      m_items.push_back(
          {c_vel, s_vact, j, Signal::Velocity, Signal::PositionActual, false});
      m_items.push_back({c_foll, s_foll, j, Signal::FollowingError,
                         Signal::PositionActual, false});
    }
    break;
  }

  case ViewMode::Overlay: {
    const Signal sig = signalAt(m_signal_combo);
    const int c = add_chart(signalName(sig), signalIsSymmetric(sig), 1);
    for (int j : joints) {
      const QString jn =
          (j < m_joint_names.size()) ? m_joint_names[j] : QString::number(j);
      const int s = m_charts[c]->addSeries(jn, jointColor(j));
      m_items.push_back({c, s, j, sig, Signal::PositionActual, false});
    }
    break;
  }

  case ViewMode::Stacked: {
    for (int j : joints) {
      const QString jn =
          (j < m_joint_names.size()) ? m_joint_names[j] : QString::number(j);
      const int c = add_chart(tr("%1 — position [deg]").arg(jn), false, 1);
      const int s_ref =
          m_charts[c]->addSeries(tr("reference"), QColor(90, 120, 170));
      const int s_act = m_charts[c]->addSeries(tr("actual"), jointColor(j));
      m_items.push_back(
          {c, s_ref, j, Signal::PositionRef, Signal::PositionActual, false});
      m_items.push_back(
          {c, s_act, j, Signal::PositionActual, Signal::PositionActual, false});
    }
    break;
  }

  case ViewMode::XY: {
    const Signal xs = signalAt(m_x_signal_combo);
    const Signal ys = signalAt(m_y_signal_combo);
    const int c =
        add_chart(tr("%1 vs %2").arg(signalName(ys), signalName(xs)), false, 1);
    m_charts[c]->setXAxis(StripChart::XAxis::Value);
    m_charts[c]->setAxisTitles(signalName(xs), signalName(ys));
    for (int j : joints) {
      const QString jn =
          (j < m_joint_names.size()) ? m_joint_names[j] : QString::number(j);
      const int s = m_charts[c]->addSeries(jn, jointColor(j));
      m_items.push_back({c, s, j, ys, xs, true});
    }
    break;
  }
  }

  m_elapsed_ms = 0;
}

void PlotPanel::clearHistory() {
  for (auto *c : m_charts) {
    c->clearAll();
  }
  m_elapsed_ms = 0;
}

void PlotPanel::setWindowSeconds(double seconds) {
  m_window_s = seconds;
  for (auto *c : m_charts) {
    c->setWindowSeconds(seconds);
  }
}

void PlotPanel::setFrozen(bool frozen) {
  m_frozen = frozen;
  if (m_freeze_btn && m_freeze_btn->isChecked() != frozen) {
    m_freeze_btn->setChecked(frozen);
  }
}

void PlotPanel::appendSample(const TelemetryFrame &frame) {
  for (const auto &it : m_items) {
    if (it.joint < 0 ||
        static_cast<std::size_t>(it.joint) >= frame.joints.size()) {
      continue;
    }
    if (it.chart < 0 || static_cast<std::size_t>(it.chart) >= m_charts.size()) {
      continue;
    }
    const JointTelemetry &j = frame.joints[static_cast<std::size_t>(it.joint)];
    const double y = signalValue(j, it.sig);
    const double x = it.xy ? signalValue(j, it.x_sig) : frame.t_s;
    m_charts[static_cast<std::size_t>(it.chart)]->append(it.series, x, y);
  }
}

void PlotPanel::appendFrame(const TelemetryFrame &frame) {
  if (m_frozen) {
    return; // Live updates paused; keep the current window on screen.
  }

  // Accumulate time, update plots only when threshold reached (throttling).
  static uint32_t last_update_time = 0;
  const auto now_ms = static_cast<uint32_t>(frame.t_s * 1000.0);
  const uint32_t dt_ms =
      (now_ms > last_update_time) ? (now_ms - last_update_time) : 16;
  m_elapsed_ms += dt_ms;

  if (m_elapsed_ms < m_update_rate_ms) {
    return; // Skip this update; buffer for next batch
  }

  last_update_time = now_ms;
  m_elapsed_ms = 0;

  appendSample(frame);
}

} // namespace actuator_test::gui
