// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Qt6 entry point for the actuator characterisation GUI.  Mirrors the console
// tool's startup (config -> runtime profile -> capability check -> realtime)
// but hands control to the windowed application instead of the picker loop.

#include "main_window.hpp"
#include "wizard/setup_wizard.hpp"

#include "actuator_test/runtime.hpp"
#include "actuator_test/settings.hpp"

#include <ethercat-primer/core>

#include <QApplication>
#include <QMessageBox>

#include <string>

int main(int argc, char *argv[])
{
    using namespace actuator_test;
    using namespace actuator_test::gui;

    QApplication app(argc, argv);

    const QString default_config =
        (argc > 1) ? QString::fromLocal8Bit(argv[1]) : QStringLiteral("../config/gene-000.toml");

    // Best-effort runtime profile: fall back to defaults if the config is
    // missing or unreadable (the worker reloads the real config on connect).
    RuntimeProfile profile;
    if (auto cfg = ecp::DeviceConfig::from_file(default_config.toStdString()))
    {
        profile = load_runtime_profile(*cfg);
    }

    register_signal_handlers();

    const std::string exe = (argc > 0) ? std::string(argv[0]) : std::string("./actuator-test-gui");
    if (!ensure_runtime_capabilities(exe, profile))
    {
        QMessageBox::warning(
            nullptr, QObject::tr("Missing capabilities"),
            QObject::tr("This program lacks the EtherCAT/realtime capabilities and will fail to connect.\n\n"
                        "Grant them with:\n  %1")
                .arg(QString::fromStdString(capability_fix_command(exe))));
    }

    if (ecp::rt_app_t::instance().init() == 0)
    {
        try_enable_realtime_scheduler(profile);
    }

    SetupWizard wizard(default_config);
    QString config = default_config;
    bool auto_connect = false;
    if (wizard.exec() == QDialog::Accepted)
    {
        config = wizard.configPath();
        auto_connect = true;
    }

    MainWindow window(profile, config);
    window.show();
    if (auto_connect)
    {
        window.requestConnect(config);
    }

    return app.exec();
}
