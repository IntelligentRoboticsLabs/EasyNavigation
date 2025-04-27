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
/// \brief Declaration of the DummyPlanner method.

#ifndef EASYNAV_PLANNER__DUMMYPLANNER_HPP_
#define EASYNAV_PLANNER__DUMMYPLANNER_HPP_

#include "nav_msgs/msg/path.hpp"

#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyPlanner
 * @brief A default "dummy" implementation for the Planner Method.
 *
 * This planning method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 */
class DummyPlanner : public easynav::PlannerMethodBase
{
public:
  DummyPlanner() = default;
  ~DummyPlanner() = default;

  /**
   * @brief Initialize the planning method.
   *
   * It is not required to override this method. Only if the derived class
   * requires further initialization than the provided by the base class.
   */
  virtual void on_initialize() override;

  /**
   * @brief Get the current path.
   *
   * This method should return the last path computed.
   * It should not run the planning algorithm (see update method).
   *
   * @return A TwistStamped message with the current path.
   */
  [[nodiscard]] virtual nav_msgs::msg::Path get_path() override;

  /**
   * @brief Run the path planning method and update the path.
   *
   * This method will be called by the system's PlannerNode to run the planning algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState nav_state) override;

private:
  /**
   * @brief Current robot velocity command.
   */
  nav_msgs::msg::Path path_ {};

};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYPLANNER_HPP_
