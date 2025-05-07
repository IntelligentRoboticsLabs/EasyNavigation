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
/// \brief Declaration of the abstract base class PlannerMethodBase.

#ifndef EASYNAV_CORE__PLANNERMETHODBASE_HPP_
#define EASYNAV_CORE__PLANNERMETHODBASE_HPP_

#include "nav_msgs/msg/path.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class PlannerMethodBase
 * @brief Abstract base class for path planning methods in Easy Navigation.
 *
 * This class serves as a template for implementing various
 * path planning algorithms in the Easy Navigation framework.
 *
 * The actual planning method should be implemented by extending this base class
 * and registering the derived class as a plugin with pluginlib,
 * which will be loaded at runtime in the system.
 */
class PlannerMethodBase : public MethodBase
{
public:
  /**
   * @brief Default constructor for PlannerMethodBase.
   */
  PlannerMethodBase() = default;

  /**
   * @brief Virtual destructor for PlannerMethodBase.
   *
   * This ensures proper cleanup of derived classes.
   */
  virtual ~PlannerMethodBase() = default;

  /**
   * @brief Get the current path.
   *
   * This method should return the last path computed.
   * It should not run the path planning algorithm (see update method).
   *
   * @return A Path message with the current route data.
   */
  [[nodiscard]] virtual nav_msgs::msg::Path get_path() = 0;

  void internal_update(const NavState & nav_state)
  {
    if (isTime2Run()) {
      update(nav_state);
    }
  }

protected:
  /**
   * @brief Run the path planning method and update in non real-time the navigation route.
   *
   * This method will be called by the system's PlannerNode to run the path planning algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__PLANNERMETHODBASE_HPP_
