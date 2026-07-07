// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "widgets/connection_panel.hpp"

#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

namespace actuator_test::gui {

ConnectionPanel::ConnectionPanel(QString default_config, QWidget *parent)
    : QWidget(parent) {
  auto *layout = new QVBoxLayout(this);

  auto *cfg_row = new QHBoxLayout();
  cfg_row->addWidget(new QLabel(tr("Config:")));
  m_config_edit = new QLineEdit(default_config);
  cfg_row->addWidget(m_config_edit, 1);
  auto *browse = new QPushButton(tr("..."));
  browse->setFixedWidth(32);
  cfg_row->addWidget(browse);
  layout->addLayout(cfg_row);

  auto *btn_row = new QHBoxLayout();
  m_connect_btn = new QPushButton(tr("Connect"));
  m_disconnect_btn = new QPushButton(tr("Disconnect"));
  m_disconnect_btn->setEnabled(false);
  btn_row->addWidget(m_connect_btn);
  btn_row->addWidget(m_disconnect_btn);
  layout->addLayout(btn_row);

  auto *sel_row = new QHBoxLayout();
  m_select_all_btn = new QPushButton(tr("Select all"));
  m_select_none_btn = new QPushButton(tr("Select none"));
  m_select_all_btn->setEnabled(false);
  m_select_none_btn->setEnabled(false);
  sel_row->addWidget(m_select_all_btn);
  sel_row->addWidget(m_select_none_btn);
  layout->addLayout(sel_row);

  layout->addWidget(new QLabel(tr("Joints (tick to act on):")));
  m_joint_list = new QListWidget();
  layout->addWidget(m_joint_list, 1);

  connect(browse, &QPushButton::clicked, this, [this] {
    const QString f = QFileDialog::getOpenFileName(
        this, tr("Select device config"), m_config_edit->text(),
        tr("TOML config (*.toml);;All files (*)"));
    if (!f.isEmpty()) {
      m_config_edit->setText(f);
    }
  });
  connect(m_connect_btn, &QPushButton::clicked, this,
          [this] { emit connectRequested(m_config_edit->text()); });
  connect(m_disconnect_btn, &QPushButton::clicked, this,
          &ConnectionPanel::disconnectRequested);
  connect(m_select_all_btn, &QPushButton::clicked, this, [this] {
    m_joint_list->blockSignals(true);
    for (int i = 0; i < m_joint_list->count(); ++i) {
      auto *item = m_joint_list->item(i);
      if (item->flags().testFlag(Qt::ItemIsEnabled)) {
        item->setCheckState(Qt::Checked);
      }
    }
    m_joint_list->blockSignals(false);
    emit selectionChanged();
  });
  connect(m_select_none_btn, &QPushButton::clicked, this, [this] {
    m_joint_list->blockSignals(true);
    for (int i = 0; i < m_joint_list->count(); ++i) {
      m_joint_list->item(i)->setCheckState(Qt::Unchecked);
    }
    m_joint_list->blockSignals(false);
    emit selectionChanged();
  });
  connect(m_joint_list, &QListWidget::itemChanged, this,
          [this] { emit selectionChanged(); });
}

void ConnectionPanel::setJoints(const std::vector<JointInfo> &joints) {
  // Preserve joint selection by name across reconnect/refresh.
  m_selected_joint_names.clear();
  for (int i = 0; i < m_joint_list->count(); ++i) {
    const auto *item = m_joint_list->item(i);
    if (item->checkState() == Qt::Checked) {
      m_selected_joint_names << item->data(Qt::UserRole).toString();
    }
  }

  m_joint_list->blockSignals(true);
  m_joint_list->clear();
  for (const auto &j : joints) {
    QString label = QString::fromStdString(j.name) + "  [" +
                    QString::fromStdString(j.driver_name) + ", " +
                    QString::fromStdString(j.operation_mode_name) + "]";
    if (!j.selectable) {
      label += tr("  (unavailable: %1)")
                   .arg(QString::fromStdString(j.unavailable_reason));
    }
    auto *item = new QListWidgetItem(label, m_joint_list);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    const QString joint_name = QString::fromStdString(j.name);
    item->setData(Qt::UserRole, joint_name);
    item->setCheckState(m_selected_joint_names.contains(joint_name)
                            ? Qt::Checked
                            : Qt::Unchecked);
    if (!j.selectable) {
      item->setFlags(item->flags() & ~Qt::ItemIsEnabled);
    }
  }
  m_joint_list->blockSignals(false);
  const bool has_any = (m_joint_list->count() > 0);
  m_select_all_btn->setEnabled(has_any);
  m_select_none_btn->setEnabled(has_any);
  emit selectionChanged();
}

void ConnectionPanel::setState(ControllerState state) {
  const bool connected = (state != ControllerState::Disconnected);
  m_connect_btn->setEnabled(!connected);
  m_disconnect_btn->setEnabled(connected);
  m_config_edit->setEnabled(!connected);
}

std::vector<std::size_t> ConnectionPanel::selectedIndices() const {
  std::vector<std::size_t> out;
  for (int i = 0; i < m_joint_list->count(); ++i) {
    if (m_joint_list->item(i)->checkState() == Qt::Checked) {
      out.push_back(static_cast<std::size_t>(i));
    }
  }
  return out;
}

QStringList ConnectionPanel::selectedJointNames() const {
  QStringList out;
  for (int i = 0; i < m_joint_list->count(); ++i) {
    const auto *item = m_joint_list->item(i);
    if (item->checkState() == Qt::Checked) {
      out << item->data(Qt::UserRole).toString();
    }
  }
  return out;
}

QString ConnectionPanel::configPath() const { return m_config_edit->text(); }

void ConnectionPanel::setConfigPath(const QString &path) {
  m_config_edit->setText(path);
}

void ConnectionPanel::setSelectedJointNames(const QStringList &names) {
  m_joint_list->blockSignals(true);
  for (int i = 0; i < m_joint_list->count(); ++i) {
    auto *item = m_joint_list->item(i);
    const QString name = item->data(Qt::UserRole).toString();
    item->setCheckState(names.contains(name) ? Qt::Checked : Qt::Unchecked);
  }
  m_joint_list->blockSignals(false);
  emit selectionChanged();
}

} // namespace actuator_test::gui
