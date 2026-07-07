// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause
//
// Adaptive update scheduler to control per-panel refresh rates and reduce
// unnecessary GUI repaints. Each panel can request its own update interval.

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace actuator_test::gui {

/// Manages update scheduling for different UI components, allowing each panel
/// to have its own refresh rate independent of the main poll timer.
class UpdateScheduler {
public:
  using UpdateFunction = std::function<void()>;

  /// Register a panel with a desired update interval (ms).
  /// Returns a unique ID for later reference.
  uint32_t registerPanel(const std::string &name, uint32_t interval_ms,
                         UpdateFunction callback);

  /// Unregister a panel by ID.
  void unregisterPanel(uint32_t panel_id);

  /// Update all eligible panels based on elapsed time.
  /// Call this from the main poll timer (can still be 60 Hz).
  void poll(uint32_t elapsed_ms);

  /// Set a new interval for a registered panel.
  void setInterval(uint32_t panel_id, uint32_t interval_ms);

  /// Get current interval for a panel.
  uint32_t getInterval(uint32_t panel_id) const;

  /// Get the name of a registered panel.
  std::string getPanelName(uint32_t panel_id) const;

  /// Reset all accumulated time (useful when pausing/resuming).
  void reset();

  /// Get all registered panel IDs.
  std::vector<uint32_t> registeredPanels() const;

private:
  struct PanelEntry {
    std::string name;
    uint32_t interval_ms = 0;
    uint32_t elapsed_ms = 0;
    UpdateFunction callback;
  };

  uint32_t m_next_id = 1;
  std::map<uint32_t, PanelEntry> m_panels;
};

} // namespace actuator_test::gui
