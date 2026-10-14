// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "core/no_load_dataset.hpp"

#include <QSaveFile>
#include <QTextStream>

namespace actuator_test::gui {

bool saveNoLoadCsv(const QString &path, NoLoadCsvSource source,
                   const NoLoadDatasetView &dataset, QString &error) {
  error.clear();
  QSaveFile file(path);
  if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
    error = file.errorString();
    return false;
  }
  QTextStream out(&file);
  out.setRealNumberPrecision(17);
  out << dataset.metadata << "# interrupted=" << (dataset.interrupted ? 1 : 0)
      << "\n# daq_dropped_samples=" << dataset.daq_dropped
      << "\n# drive_dropped_samples=" << dataset.drive_dropped
      << "\n# drive_late_interval_threshold_s=" << 1.5 * dataset.drive_period_s
      << "\n# drive_late_intervals=" << dataset.drive_late_intervals
      << "\n# drive_max_host_interval_s=" << dataset.drive_max_host_interval_s
      << "\n# result="
      << QString(dataset.result).replace('\n', QStringLiteral("\n# "))
      << "\n# synchronization=independent clocks; synchronize in "
         "post-processing\n";

  switch (source) {
  case NoLoadCsvSource::Supply:
    out << "# source=instrument ELOG samples\n"
           "elog_time_s,supply_voltage_v,supply_current_a\n";
    for (const auto &sample : dataset.supply_samples)
      out << sample.t_s << ',' << sample.voltage_v << ',' << sample.current_a
          << '\n';
    break;
  case NoLoadCsvSource::Drive:
    out << "# source=buffered EtherCAT telemetry; nominal 100 Hz\n"
           "# host_time_s=host steady clock at telemetry publication; "
           "cycle time is nominal, inspect host intervals for jitter\n"
           "drive_time_s,drive_speed_rpm,cycle_index,host_time_s\n";
    for (const auto &sample : dataset.drive_samples)
      if (sample.joint == dataset.joint)
        out << sample.t_s << ',' << sample.speed_rpm << ','
            << sample.cycle_index << ',' << sample.host_time_s << '\n';
    break;
  case NoLoadCsvSource::Sensor:
    out << "# source=buffered DAQ block results; 100 Hz; not raw 50 kHz "
           "stream\n"
           "# daq_time_s=hardware sample count / analog sampling rate\n"
           "daq_time_s,external_raw_torque_nm,external_filtered_torque_nm,"
           "external_speed_rpm,sample_index,delivery_time_s\n";
    for (const auto &sample : dataset.sensor_samples)
      out << sample.t_s << ',' << sample.torque_nm << ','
          << sample.filtered_torque_nm << ',' << sample.speed_rpm << ','
          << sample.sample_index << ',' << sample.delivery_time_s << '\n';
    break;
  }
  out.flush();
  if (out.status() != QTextStream::Ok) {
    error = file.errorString();
    return false;
  }
  if (!file.commit()) {
    error = file.errorString();
    return false;
  }
  return true;
}

} // namespace actuator_test::gui
