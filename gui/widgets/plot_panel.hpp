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
#include <QList>
#include <QPointF>
#include <QString>
#include <QStringList>
#include <QWidget>
#include <deque>
#include <vector>

QT_BEGIN_NAMESPACE
class QComboBox;
class QPushButton;
class QCheckBox;
class QLabel;
class QVBoxLayout;
QT_END_NAMESPACE

namespace actuator_test::gui {

/// A minimal rolling-window multi-series strip chart.  Not a QObject: it has no
/// signals/slots, just custom painting.
class StripChart : public QWidget {
public:
  /// How the horizontal axis is interpreted.
  enum class XAxis {
    Time,  ///< X is seconds; points scroll out of a fixed time window.
    Value, ///< X is an arbitrary signal value (phase / XY plots).
  };

  explicit StripChart(QString title, QWidget *parent = nullptr);

  /// Register a named series and return its index.
  int addSeries(const QString &name, const QColor &color);

  /// Remove every series (definitions and data).
  void resetSeries();

  /// Append a sample to a series (x is seconds in Time mode).
  void append(int series, double x, double y);

  /// Force the vertical axis to be symmetric about zero (used for error).
  void setSymmetric(bool symmetric);

  void setWindowSeconds(double seconds);
  void setXAxis(XAxis axis) { m_x_axis = axis; }
  void setAxisTitles(const QString &x, const QString &y);
  void clearAll();

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  struct Series {
    QString name;
    QColor color;
    std::deque<QPointF> points;
  };

  QString m_title;
  QString m_x_title;
  QString m_y_title;
  std::vector<Series> m_series;
  double m_window_s = 10.0;
  bool m_symmetric = false;
  double m_latest_x = 0.0;
  XAxis m_x_axis = XAxis::Time;
  std::size_t m_max_points = 2000;
};

/// The dockable plot panel.  Supports several view modes so multiple devices'
/// signals can be compared: focus (one joint, full detail), overlay (one signal
/// across many joints), stacked (one chart per joint) and XY phase plots.
class PlotPanel : public QWidget {
  Q_OBJECT

public:
  explicit PlotPanel(QWidget *parent = nullptr);

  void setJoints(const QStringList &names);
  void appendFrame(const TelemetryFrame &frame);
  void clearHistory();

  /// Set the update rate for plot refresh (milliseconds between updates).
  void setUpdateRate(uint32_t ms) { m_update_rate_ms = ms; }

  /// Set the time window displayed by the charts (seconds).
  void setWindowSeconds(double seconds);

  /// Freeze/unfreeze live updates (frozen charts keep their current window).
  void setFrozen(bool frozen);
  bool isFrozen() const { return m_frozen; }

private:
  /// Selectable per-joint signals.
  enum class Signal {
    PositionActual,
    PositionRef,
    Error,
    Velocity,
    RefVelocity,
    FollowingError,
  };

  /// Overall layout of the plotting area.
  enum class ViewMode {
    Focus,   ///< One joint, four detailed charts.
    Overlay, ///< One chart, one signal per selected joint.
    Stacked, ///< One position chart per selected joint.
    XY,      ///< Phase plot: chosen X signal vs Y signal.
  };

  void rebuildCharts();
  void appendSample(const TelemetryFrame &frame);
  std::vector<int> checkedJoints() const;
  static double signalValue(const JointTelemetry &j, Signal s);
  static bool signalIsSymmetric(Signal s);
  static QColor jointColor(int index);
  static QString signalName(Signal s);
  Signal signalAt(const QComboBox *combo) const;

  /// One line to draw: which chart/series, which joint, and which signal(s).
  struct DrawItem {
    int chart = 0;
    int series = 0;
    int joint = 0;
    Signal sig =
        Signal::PositionActual; ///< Y signal (or value for time plots).
    Signal x_sig = Signal::PositionActual; ///< X signal (XY mode only).
    bool xy = false;
  };

  // Controls.
  QComboBox *m_mode_combo = nullptr;
  QComboBox *m_signal_combo = nullptr;
  QComboBox *m_x_signal_combo = nullptr;
  QComboBox *m_y_signal_combo = nullptr;
  QLabel *m_signal_label = nullptr;
  QLabel *m_x_signal_label = nullptr;
  QLabel *m_y_signal_label = nullptr;
  QComboBox *m_window_combo = nullptr;
  QPushButton *m_freeze_btn = nullptr;
  QList<QCheckBox *> m_joint_checks;
  QWidget *m_joint_checks_host = nullptr;
  QVBoxLayout *m_charts_layout = nullptr;
  QWidget *m_charts_container = nullptr;

  // Active chart set (rebuilt on mode / selection change).
  std::vector<StripChart *> m_charts;
  std::vector<DrawItem> m_items;

  QStringList m_joint_names;
  ViewMode m_mode = ViewMode::Focus;
  double m_window_s = 10.0;
  uint32_t m_update_rate_ms = 50;
  uint32_t m_elapsed_ms = 0;
  bool m_frozen = false;
};

} // namespace actuator_test::gui
