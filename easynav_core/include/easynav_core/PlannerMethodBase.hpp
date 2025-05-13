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
 * This class defines the interface for implementing planning algorithms.
 * Derived classes must implement the update and get_path methods.
 */
class PlannerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  PlannerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~PlannerMethodBase() = default;

  /**
   * @brief Get the current path.
   *
   * Should return the last computed path without triggering a new computation.
   *
   * @return A Path message representing the current planned path.
   */
  [[nodiscard]] virtual nav_msgs::msg::Path get_path() = 0;

  /**
   * @brief Helper to run the planner update if it is time.
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
   * @brief Run the path planning algorithm and update the route.
   *
   * Called periodically by the system to compute or refine a navigation path.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__PLANNERMETHODBASE_HPP_
