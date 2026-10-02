// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/power_supply.hpp"

#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <thread>

int main(int argc, char **argv) {
  if (argc != 6) {
    return 2;
  }
  actuator_test::PowerSupplyConfig config;
  config.host = "127.0.0.1";
  config.port = static_cast<uint16_t>(std::stoi(argv[1]));
  config.duration_s = std::stod(argv[2]);
  config.timeout_s = 0.2;
  config.download_timeout_s = std::stod(argv[4]);
  config.sample_period_s = std::stod(argv[5]);
  const double stop_after = std::stod(argv[3]);
  actuator_test::PowerSupplyReader reader(config);
  reader.start();
  const auto start = std::chrono::steady_clock::now();
  while (!reader.finished()) {
    const double elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - start)
            .count();
    if (stop_after == -2 && elapsed >= 0.35) {
      return 0; // Exercise destruction during an unfinished download.
    }
    if (stop_after >= 0 && elapsed >= stop_after) {
      reader.requestStop();
    }
    if (elapsed > 10.0) {
      std::cerr << "test acquisition did not finish\n";
      return 3;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::cout << reader.status() << '\n';
  std::cout << "interrupted=" << reader.interrupted() << '\n';
  std::cout << std::setprecision(17);
  for (const auto &sample : reader.takeSamples()) {
    std::cout << sample.t_s << ',' << sample.voltage_v << ','
              << sample.current_a << ',' << sample.power_w << '\n';
  }
}
