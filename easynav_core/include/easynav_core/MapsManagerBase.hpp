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
/// \brief Declaration of the abstract base class MapsManagerBase.

#ifndef EASYNAV_CORE__MAPSMANAGERBASE_HPP_
#define EASYNAV_CORE__MAPSMANAGERBASE_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class MapsManagerBase
 * @brief Abstract base class for map management in Easy Navigation.
 *
 * This class defines the interface for components responsible for generating or maintaining maps.
 * Derived classes must implement the update and get_maps methods.
 */
class MapsManagerBase : public MethodBase
{
public:
  /// @brief Default constructor.
  MapsManagerBase() = default;

  /// @brief Virtual destructor.
  virtual ~MapsManagerBase() = default;

  /**
   * @brief Get the current set of maps.
   *
   * Should return the last generated or updated maps, without performing computation.
   *
   * @return A map from identifiers to map representations.
   */
  [[nodiscard]] virtual std::map<std::string, std::shared_ptr<MapsTypeBase>> get_maps() = 0;

  /**
   * @brief Helper to run the update method if it is time to do so.
   *
   * @param nav_state The current state of the navigation system.
   */
  void internal_update(const NavState & nav_state)
  {
    if (isTime2Run()) {
      update(nav_state);
    }
  }

protected:
  /**
   * @brief Run the map update logic.
   *
   * Called periodically by the system to update map data using the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__MAPSMANAGERBASE_HPP_
