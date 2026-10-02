// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "actuator_test/power_supply.hpp"

#include <arpa/inet.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <charconv>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <string_view>

namespace actuator_test {
namespace {

using Clock = std::chrono::steady_clock;

Clock::time_point deadlineAfter(double seconds) {
  return Clock::now() + std::chrono::duration_cast<Clock::duration>(
                            std::chrono::duration<double>(seconds));
}

std::string_view trim(std::string_view text) {
  const auto first = text.find_first_not_of(" \t\r\n");
  if (first == std::string_view::npos) {
    return {};
  }
  return text.substr(first, text.find_last_not_of(" \t\r\n") - first + 1);
}

double parseNumber(std::string_view text) {
  text = trim(text);
  if (!text.empty() && text.front() == '+') {
    text.remove_prefix(1);
  }
  if (text.empty()) {
    throw std::runtime_error("empty ELOG number");
  }
  double value = 0.0;
  const auto [end, error] =
      std::from_chars(text.data(), text.data() + text.size(), value);
  if (error != std::errc{} || end != text.data() + text.size() ||
      !std::isfinite(value)) {
    throw std::runtime_error("invalid ELOG number: " + std::string(text));
  }
  return value;
}

std::string scpiNumber(double value) {
  char buffer[64];
  const auto [end, error] =
      std::to_chars(buffer, buffer + sizeof(buffer), value);
  if (error != std::errc{}) {
    throw std::runtime_error("cannot format ELOG configuration");
  }
  return std::string(buffer, end);
}

std::vector<PowerSupplyReader::Sample> parseSamples(std::string_view data,
                                                    double period) {
  std::vector<PowerSupplyReader::Sample> samples;
  data = trim(data);
  while (!data.empty()) {
    const auto separator = data.find(';');
    const auto record = trim(data.substr(0, separator));
    const auto comma = record.find(',');
    if (comma == std::string_view::npos) {
      throw std::runtime_error("invalid ELOG record: " + std::string(record));
    }
    const double voltage = parseNumber(record.substr(0, comma));
    const double current = parseNumber(record.substr(comma + 1));
    const double power = voltage * current;
    if (!std::isfinite(power)) {
      throw std::runtime_error("non-finite ELOG power");
    }
    samples.push_back({samples.size() * period, voltage, current, power});
    if (separator == std::string_view::npos) {
      break;
    }
    data = trim(data.substr(separator + 1));
  }
  return samples;
}

} // namespace

PowerSupplyReader::PowerSupplyReader(PowerSupplyConfig cfg)
    : m_cfg(std::move(cfg)) {
  if (!std::isfinite(m_cfg.sample_period_s) || m_cfg.sample_period_s < 0.0001 ||
      m_cfg.sample_period_s > 100.0 || !std::isfinite(m_cfg.duration_s) ||
      m_cfg.duration_s < m_cfg.sample_period_s ||
      !std::isfinite(m_cfg.timeout_s) || m_cfg.timeout_s <= 0.0 ||
      !std::isfinite(m_cfg.download_timeout_s) ||
      m_cfg.download_timeout_s <= 0.0) {
    throw std::invalid_argument("invalid ELOG period, duration or timeout");
  }
}

PowerSupplyReader::~PowerSupplyReader() {
  m_cancel = true;
  requestStop();
  if (m_thread.joinable()) {
    m_thread.join();
  }
}

void PowerSupplyReader::start() {
  if (!m_thread.joinable()) {
    setStatus("Connecting to ITECH...");
    m_thread = std::thread([this] { run(); });
  }
}

void PowerSupplyReader::requestStop() { m_stop_requested = true; }

std::vector<PowerSupplyReader::Sample> PowerSupplyReader::takeSamples() {
  if (!finished()) {
    throw std::logic_error("ELOG download is not finished");
  }
  return std::move(m_samples);
}

std::string PowerSupplyReader::status() const {
  std::lock_guard lock(m_mutex);
  return m_status;
}

void PowerSupplyReader::setStatus(std::string status) {
  std::lock_guard lock(m_mutex);
  m_status = std::move(status);
}

void PowerSupplyReader::sleepFor(double seconds) const {
  const auto deadline = deadlineAfter(seconds);
  while (!m_stop_requested && !m_cancel && Clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void PowerSupplyReader::waitSocket(short events, Clock::time_point deadline) {
  while (!m_cancel) {
    if (Clock::now() >= deadline) {
      throw std::runtime_error("SCPI communication timeout");
    }
    pollfd fd{m_fd, events, 0};
    const int result = ::poll(&fd, 1, 50);
    if (result > 0) {
      if (fd.revents & (events | POLLHUP)) {
        return;
      }
      throw std::runtime_error("SCPI socket error");
    }
    if (result < 0 && errno != EINTR) {
      throw std::runtime_error("poll failed: " +
                               std::string(std::strerror(errno)));
    }
  }
  throw std::runtime_error("ELOG download cancelled during shutdown");
}

void PowerSupplyReader::connectSocket() {
  sockaddr_in address{};
  address.sin_family = AF_INET;
  address.sin_port = htons(m_cfg.port);
  if (::inet_pton(AF_INET, m_cfg.host.c_str(), &address.sin_addr) != 1) {
    throw std::runtime_error("invalid supply IPv4 address: " + m_cfg.host);
  }
  m_fd = ::socket(AF_INET, SOCK_STREAM | SOCK_CLOEXEC, 0);
  if (m_fd < 0 || ::fcntl(m_fd, F_SETFL, O_NONBLOCK) < 0) {
    throw std::runtime_error("socket setup failed: " +
                             std::string(std::strerror(errno)));
  }
  if (::connect(m_fd, reinterpret_cast<sockaddr *>(&address),
                sizeof(address)) != 0) {
    if (errno != EINPROGRESS) {
      throw std::runtime_error("connect failed: " +
                               std::string(std::strerror(errno)));
    }
    waitSocket(POLLOUT, deadlineAfter(m_cfg.timeout_s));
    int error = 0;
    socklen_t size = sizeof(error);
    if (::getsockopt(m_fd, SOL_SOCKET, SO_ERROR, &error, &size) != 0) {
      throw std::runtime_error("connect status failed: " +
                               std::string(std::strerror(errno)));
    }
    if (error != 0) {
      throw std::runtime_error("connect failed: " +
                               std::string(std::strerror(error)));
    }
  }
}

void PowerSupplyReader::closeSocket() {
  if (m_fd >= 0) {
    ::close(m_fd);
    m_fd = -1;
  }
}

void PowerSupplyReader::command(const std::string &text) {
  const std::string line = text + "\n";
  const auto deadline = deadlineAfter(m_cfg.timeout_s);
  std::size_t sent = 0;
  while (sent < line.size()) {
    waitSocket(POLLOUT, deadline);
    const auto n =
        ::send(m_fd, line.data() + sent, line.size() - sent, MSG_NOSIGNAL);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      continue;
    }
    if (n <= 0) {
      throw std::runtime_error("SCPI send failed: " +
                               std::string(std::strerror(errno)));
    }
    sent += static_cast<std::size_t>(n);
  }
}

std::string PowerSupplyReader::query(const std::string &text,
                                     double timeout_s) {
  command(text);
  const auto deadline = deadlineAfter(timeout_s);
  std::string response;
  char buffer[4096];
  for (;;) {
    waitSocket(POLLIN, deadline);
    const auto n = ::recv(m_fd, buffer, sizeof(buffer), 0);
    if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)) {
      continue;
    }
    if (n <= 0) {
      throw std::runtime_error(n == 0 ? "ITECH connection closed"
                                      : "SCPI receive failed: " +
                                            std::string(std::strerror(errno)));
    }
    const auto previous_size = response.size();
    response.append(buffer, static_cast<std::size_t>(n));
    if (response.size() > 128 * 1024 * 1024) {
      throw std::runtime_error(
          "ELOG response exceeds software limit (128 MiB)");
    }
    const auto lf = response.find('\n', previous_size);
    if (lf != std::string::npos) {
      if (lf + 1 != response.size()) {
        throw std::runtime_error(
            "unexpected data after SCPI response terminator");
      }
      response.resize(lf);
      if (!response.empty() && response.back() == '\r') {
        response.pop_back();
      }
      return response;
    }
  }
}

void PowerSupplyReader::run() {
  bool elog_active = false;
  try {
    connectSocket();
    const auto id = query("*IDN?", m_cfg.timeout_s);
    if (id.find("ITECH Electronics,IT-M3902C-80-40,") != 0) {
      throw std::runtime_error("unexpected instrument: " + id);
    }
    if (m_stop_requested) {
      throw std::runtime_error("ELOG stopped before initialization");
    }
    command("SYST:REM");
    command("ABOR:ELOG");
    command("SENS:ELOG:PER " + scpiNumber(m_cfg.sample_period_s));
    command("SENS:ELOG:TIME " + scpiNumber(m_cfg.duration_s));
    command("SENS:ELOG:FUNC:VOLT 1");
    command("SENS:ELOG:FUNC:CURR 1");
    command("SENS:ELOG:FUNC:PEAK 0");
    command("FORMAT ASCII");
    command("TRIG:ELOG:SOUR MANUAL");
    command("INIT:ELOG");
    elog_active = true;
    setStatus("ELOG: waiting for trigger readiness...");
    const auto arm_deadline = deadlineAfter(5.0);
    while (!m_stop_requested) {
      const auto state = query("ELOG:STAT?", m_cfg.timeout_s);
      if (state == "wait trigger") {
        command("TRIG:ELOG");
        break;
      }
      if (state != "idle") {
        throw std::runtime_error("unexpected ELOG arming state: " + state);
      }
      if (Clock::now() >= arm_deadline) {
        throw std::runtime_error("ELOG trigger readiness timeout");
      }
      sleepFor(0.05);
    }
    setStatus("ELOG: acquiring internally...");
    const auto acquisition_deadline = deadlineAfter(m_cfg.duration_s + 5.0);
    while (!m_stop_requested) {
      const auto state = query("ELOG:STAT?", m_cfg.timeout_s);
      if (state == "idle") {
        break;
      }
      if (state != "action" && state != "wait trigger") {
        throw std::runtime_error("unexpected ELOG acquisition state: " + state);
      }
      if (Clock::now() >= acquisition_deadline) {
        throw std::runtime_error("ELOG completion timeout");
      }
      sleepFor(0.1);
    }
    if (m_stop_requested) {
      m_interrupted = true;
      command("ABOR:ELOG");
      const auto abort_deadline = deadlineAfter(5.0);
      while (query("ELOG:STAT?", m_cfg.timeout_s) != "idle") {
        if (Clock::now() >= abort_deadline) {
          throw std::runtime_error("ELOG abort timeout");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
      }
    }
    elog_active = false;
    setStatus("ELOG: downloading ASCII data...");
    auto samples =
        parseSamples(query("FETC:ELOG:ARR:DATA?", m_cfg.download_timeout_s),
                     m_cfg.sample_period_s);
    const double expected = m_cfg.duration_s / m_cfg.sample_period_s;
    std::string status =
        m_interrupted ? "ELOG interrupted: " : "ELOG complete: ";
    status += std::to_string(samples.size()) + " samples";
    if (!m_interrupted && std::fabs(samples.size() - expected) > 1.0) {
      status +=
          " (WARNING: expected approximately " + scpiNumber(expected) + ")";
    }
    if (samples.empty()) {
      status += " (WARNING: empty buffer)";
    }
    m_samples = std::move(samples);
    setStatus(std::move(status));
  } catch (const std::exception &error) {
    std::string status =
        "ELOG error: " + std::string(error.what()) +
        ". No complete dataset downloaded; start a new test to reconnect.";
    // Never replay INIT/TRIG after a disconnect: that would silently create
    // a different acquisition. A still-open socket can stop the finite ELOG.
    if (elog_active && !m_cancel) {
      try {
        command("ABOR:ELOG");
      } catch (const std::exception &abort_error) {
        status += " Abort failed: " + std::string(abort_error.what());
      }
    }
    setStatus(std::move(status));
  }
  closeSocket();
  m_finished = true;
}

} // namespace actuator_test
