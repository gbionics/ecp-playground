// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Bus connection + joint selection.  Owns the canonical "which joints are
// selected" checkable list that the limit and trajectory panels act upon.

#pragma once

#include "core/controller_worker.hpp"

#include <QStringList>
#include <QWidget>
#include <vector>

QT_BEGIN_NAMESPACE
class QLineEdit;
class QPushButton;
class QListWidget;
QT_END_NAMESPACE

namespace actuator_test::gui {

class ConnectionPanel : public QWidget {
  Q_OBJECT

public:
  explicit ConnectionPanel(QString default_config, QWidget *parent = nullptr);

  /// Populate / refresh the joint list after a (dis)connect.
  void setJoints(const std::vector<JointInfo> &joints);

  /// Reflect the controller state (enables/disables buttons).
  void setState(ControllerState state);

  /// Indices of the joints the user has ticked.
  std::vector<std::size_t> selectedIndices() const;
  QStringList selectedJointNames() const;

  QString configPath() const;
  void setConfigPath(const QString &path);
  void setSelectedJointNames(const QStringList &names);

signals:
  void connectRequested(const QString &config_path);
  void disconnectRequested();
  void selectionChanged();

private:
  QLineEdit *m_config_edit = nullptr;
  QPushButton *m_connect_btn = nullptr;
  QPushButton *m_disconnect_btn = nullptr;
  QPushButton *m_select_all_btn = nullptr;
  QPushButton *m_select_none_btn = nullptr;
  QListWidget *m_joint_list = nullptr;
  QStringList m_selected_joint_names;
};

} // namespace actuator_test::gui
