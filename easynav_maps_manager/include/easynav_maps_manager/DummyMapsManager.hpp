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
 * Serves as a placeholder or fallback when no real map manager is provided.
 */
class DummyMapsManager : public easynav::MapsManagerBase
{
public:
  /// @brief Default constructor.
  DummyMapsManager() = default;

  /// @brief Default destructor.
  ~DummyMapsManager() = default;

  /**
   * @brief Initialize the plugin.
   * @return Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Return the current maps.
   * @return An empty map.
   */
  std::map<std::string, std::shared_ptr<MapsTypeBase>> get_maps()
  {
    return std::map<std::string, std::shared_ptr<MapsTypeBase>>();
  }

  /**
   * @brief Dummy update method.
   * @param nav_state The current navigation state.
   */
  virtual void update(const NavState & nav_state) override;

private:
  double cycle_time_rt_;
  double cycle_time_nort_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYMAPMANAGER_HPP_
