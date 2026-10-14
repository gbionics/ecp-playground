// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include "actuator_test/external_daq.hpp"
#include "actuator_test/power_supply.hpp"
#include "core/telemetry.hpp"

#include <QString>
#include <span>

namespace actuator_test::gui {

enum class NoLoadCsvSource { Sensor, Supply, Drive };

struct NoLoadDatasetView {
  QString metadata;
  QString result;
  bool interrupted = false;
  std::size_t joint = 0;
  std::size_t daq_dropped = 0;
  std::size_t drive_dropped = 0;
  double drive_period_s = 0;
  std::size_t drive_late_intervals = 0;
  double drive_max_host_interval_s = 0;
  std::span<const ExternalDaqReader::Sample> sensor_samples;
  std::span<const PowerSupplyReader::Sample> supply_samples;
  std::span<const DriveSample> drive_samples;
};

bool saveNoLoadCsv(const QString &path, NoLoadCsvSource source,
                   const NoLoadDatasetView &dataset, QString &error);

} // namespace actuator_test::gui
