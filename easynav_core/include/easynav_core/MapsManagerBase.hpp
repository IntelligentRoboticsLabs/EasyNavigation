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
 * @brief Abstract base class for localization methods in Easy Navigation.
 *
 * This class serves as a template for implementing various localization
 * algorithms in the Easy Navigation framework.
 *
 * The actual localization method should be implemented by extending this base class
 * and registering the derived class as a plugin with pluginlib,
 * which will be loaded at runtime in the system.
 */
class MapsManagerBase : public MethodBase
{
public:
  /**
   * @brief Default constructor for MapsManagerBase.
   */
  MapsManagerBase() = default;

  /**
   * @brief Virtual destructor for MapsManagerBase.
   *
   * This ensures proper cleanup of derived classes.
   */
  virtual ~MapsManagerBase() = default;

  /**
   * @brief Get the current localization state.
   *
   * This method should return the last known localization of the robot.
   * It should not run the localization algorithm (see update method).
   *
   * @return An Odometry message representing the current localization state.
   */
  [[nodiscard]] virtual std::shared_ptr<MapsTypeBase> get_static_map() = 0;

  /**
   * @brief Get the current localization state.
   *
   * This method should return the last known localization of the robot.
   * It should not run the localization algorithm (see update method).
   *
   * @return An Odometry message representing the current localization state.
   */
  [[nodiscard]] virtual std::shared_ptr<MapsTypeBase> get_dynamyc_map() = 0;

  void internal_update(const NavState & nav_state)
  {
    if (isTime2Run()) {
      update(nav_state);
    }
  }

protected:
  /**
   * @brief Run the localization method and update in non real-time the robot's estimated localization.
   *
   * This method will be called by the system's LocalizerNode to run the localization algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__MAPSMANAGERBASE_HPP_
