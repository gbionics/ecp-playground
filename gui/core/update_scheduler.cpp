// SPDX-FileCopyrightText: Generative Bionics S.R.L.
// SPDX-License-Identifier: BSD-3-Clause

#include "update_scheduler.hpp"

namespace actuator_test::gui
{

uint32_t UpdateScheduler::registerPanel(const std::string &name, uint32_t interval_ms,
                                        UpdateFunction callback)
{
    auto id = m_next_id++;
    m_panels[id] = PanelEntry{name, interval_ms, 0, std::move(callback)};
    return id;
}

void UpdateScheduler::unregisterPanel(uint32_t panel_id)
{
    m_panels.erase(panel_id);
}

void UpdateScheduler::poll(uint32_t elapsed_ms)
{
    for (auto &[id, entry] : m_panels)
    {
        entry.elapsed_ms += elapsed_ms;
        if (entry.elapsed_ms >= entry.interval_ms)
        {
            entry.elapsed_ms = 0;
            if (entry.callback)
            {
                entry.callback();
            }
        }
    }
}

void UpdateScheduler::setInterval(uint32_t panel_id, uint32_t interval_ms)
{
    auto it = m_panels.find(panel_id);
    if (it != m_panels.end())
    {
        it->second.interval_ms = interval_ms;
    }
}

uint32_t UpdateScheduler::getInterval(uint32_t panel_id) const
{
    auto it = m_panels.find(panel_id);
    if (it != m_panels.end())
    {
        return it->second.interval_ms;
    }
    return 0;
}

std::string UpdateScheduler::getPanelName(uint32_t panel_id) const
{
    auto it = m_panels.find(panel_id);
    if (it != m_panels.end())
    {
        return it->second.name;
    }
    return "";
}

void UpdateScheduler::reset()
{
    for (auto &[id, entry] : m_panels)
    {
        entry.elapsed_ms = 0;
    }
}

std::vector<uint32_t> UpdateScheduler::registeredPanels() const
{
    std::vector<uint32_t> result;
    result.reserve(m_panels.size());
    for (const auto &[id, entry] : m_panels)
    {
        result.push_back(id);
    }
    return result;
}

} // namespace actuator_test::gui
