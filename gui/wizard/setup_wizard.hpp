// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// A short onboarding wizard that explains the characterisation workflow and
// collects the device-config path before the main window connects to the bus.

#pragma once

#include <QWizard>

QT_BEGIN_NAMESPACE
class QLineEdit;
QT_END_NAMESPACE

namespace actuator_test::gui
{

class SetupWizard : public QWizard
{
    Q_OBJECT

public:
    explicit SetupWizard(QString default_config, QWidget *parent = nullptr);

    /// The config path chosen on the connection page.
    QString configPath() const;

private:
    QLineEdit *m_config_edit = nullptr;
};

} // namespace actuator_test::gui
