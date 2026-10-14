// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#pragma once

#include <cstddef>
#include <mutex>
#include <vector>

namespace actuator_test {

// Bounded, preallocated producer storage. Overflow is counted, never hidden.
template <typename Sample> class SampleBuffer {
public:
  explicit SampleBuffer(std::size_t capacity = 8192)
      : m_storage(capacity), m_consumer_storage(capacity) {}

  void start() {
    std::lock_guard<std::mutex> lock(m_mutex);
    m_count = m_dropped = 0;
    m_active = true;
  }

  void push(const Sample &sample) {
    std::lock_guard<std::mutex> lock(m_mutex);
    if (!m_active)
      return;
    if (m_count == m_storage.size())
      ++m_dropped;
    else
      m_storage[m_count++] = sample;
  }

  std::size_t drain(std::vector<Sample> &destination, bool stop = false) {
    std::size_t count, dropped;
    {
      std::lock_guard<std::mutex> lock(m_mutex);
      if (stop)
        m_active = false;
      count = m_count;
      dropped = m_dropped;
      m_storage.swap(m_consumer_storage);
      m_count = 0;
    }
    // One consumer only. Vector growth must never hold the producer mutex.
    destination.insert(destination.end(), m_consumer_storage.begin(),
                       m_consumer_storage.begin() + count);
    return dropped;
  }

private:
  std::mutex m_mutex;
  std::vector<Sample> m_storage;
  std::vector<Sample> m_consumer_storage;
  std::size_t m_count = 0, m_dropped = 0;
  bool m_active = false;
};

} // namespace actuator_test
