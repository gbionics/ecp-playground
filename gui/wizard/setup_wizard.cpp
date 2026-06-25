// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "wizard/setup_wizard.hpp"

#include <QFileDialog>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QVBoxLayout>
#include <QWizardPage>

namespace actuator_test::gui
{

namespace
{

QWizardPage *makeWelcomePage()
{
    auto *page = new QWizardPage();
    page->setTitle(QObject::tr("Actuator Characterisation"));
    auto *layout = new QVBoxLayout(page);
    auto *text = new QLabel(QObject::tr(
        "<p>This tool drives EtherCAT actuators through characterisation "
        "trajectories and records jitter-free 1&nbsp;kHz logs.</p>"
        "<p><b>Workflow</b></p>"
        "<ol>"
        "<li><b>Connect</b> to the bus using a device-config TOML.</li>"
        "<li><b>Select</b> the joints to exercise.</li>"
        "<li><b>Capture limits</b> by backdriving, or type them in.</li>"
        "<li><b>Jog / home</b> to verify motion safely.</li>"
        "<li><b>Play</b> a waveform (sine, chirp, triangle, step, multisine) "
        "and watch reference / actual / error live.</li>"
        "</ol>"
        "<p>Requires <tt>cap_net_raw,cap_net_admin,cap_sys_nice</tt> on the "
        "executable (see the capabilities task).</p>"));
    text->setWordWrap(true);
    layout->addWidget(text);
    return page;
}

} // namespace

SetupWizard::SetupWizard(QString default_config, QWidget *parent) : QWizard(parent)
{
    setWindowTitle(tr("Setup"));
    addPage(makeWelcomePage());

    auto *conn = new QWizardPage();
    conn->setTitle(tr("Device configuration"));
    conn->setSubTitle(tr("Choose the TOML that describes the bus and joints."));
    auto *layout = new QVBoxLayout(conn);
    auto *row = new QHBoxLayout();
    row->addWidget(new QLabel(tr("Config:")));
    m_config_edit = new QLineEdit(default_config);
    row->addWidget(m_config_edit, 1);
    auto *browse = new QPushButton(tr("..."));
    row->addWidget(browse);
    layout->addLayout(row);
    addPage(conn);

    connect(browse, &QPushButton::clicked, this, [this] {
        const QString f = QFileDialog::getOpenFileName(this, tr("Select device config"), m_config_edit->text(),
                                                       tr("TOML config (*.toml);;All files (*)"));
        if (!f.isEmpty())
        {
            m_config_edit->setText(f);
        }
    });
}

QString SetupWizard::configPath() const
{
    return m_config_edit->text();
}

} // namespace actuator_test::gui
