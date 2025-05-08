// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in short)
// licensed under the GNU General Public License v3.0.
// See <http://www.gnu.org/licenses/> for details.
//
// Easy Navigation program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

/// \file
/// \brief Declaration of the DummyMapsManager class, a default map manager plugin for Easy Navigation.

#ifndef EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_
#define EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_

#include <expected>

#include "nav_msgs/msg/path.hpp"
#include "easynav_core/MapsManagerBase.hpp"
#include "easynav_common/types/MapTypeBase.hpp"

namespace easynav
{

/**
 * @class DummyMapsManager
 * @brief A default "do-nothing" implementation of MapsManagerBase.
 *
 * This class fulfills the interface for a map manager in EasyNav but does not
 * manage or modify any actual map data. It is intended to be used as a placeholder
 * or fallback when no real implementation is specified in the configuration.
 */
class DummyMapsManager : public easynav::MapsManagerBase
{
public:
  /**
   * @brief Default constructor.
   */
  DummyMapsManager() = default;

  /**
   * @brief Default destructor.
   */
  ~DummyMapsManager() = default;

  /**
   * @brief Initialize the dummy maps manager.
   *
   * Optionally overridden in derived classes to perform setup logic.
   *
   * @return std::expected<void, std::string> Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  std::map<std::string, std::shared_ptr<MapsTypeBase>> get_maps()
  {
    return std::map<std::string, std::shared_ptr<MapsTypeBase>>();
  }

  /**
   * @brief Dummy update method.
   *
   * This function is intended to update the internal map data based on the current
   * navigation state. In this dummy implementation, it performs no action.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) override;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_
