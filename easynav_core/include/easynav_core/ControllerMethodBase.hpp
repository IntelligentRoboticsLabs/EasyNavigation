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
/// \brief Declaration of the abstract base class ControllerMethodBase.

#ifndef EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
#define EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class ControllerMethodBase
 * @brief Abstract base class for control methods in Easy Navigation.
 *
 * This class serves as a template for implementing various
 * control algorithms in the Easy Navigation framework.
 *
 * The actual planning method should be implemented by extending this base class
 * and registering the derived class as a plugin with pluginlib,
 * which will be loaded at runtime in the system.
 */
class ControllerMethodBase
{
public:
  /**
   * @brief Default constructor for ControllerMethodBase.
   */
  ControllerMethodBase() = default;

  /**
   * @brief Virtual destructor for ControllerMethodBase.
   *
   * This ensures proper cleanup of derived classes.
   */
  virtual ~ControllerMethodBase() = default;

  /**
   * @brief Initialize the control method.
   *
   * This method should be called to set up any necessary resources
   * or configurations for the control algorithm.
   * By default, it sets the parent node to the provided lifecycle node.
   *
   * @note If this method is overriden, the derived class should call the base method
   * to ensure proper initialization of the parent node.
   */
  virtual void initialize(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node);

  /**
   * @brief get a pointer to the parent lifecycle node.
   *
   * This method can be used by derived classes to access the (private) lifecycle node.
   *
   * @return A shared pointer to the parent lifecycle node.
   */
  [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode> get_node() const;

  /**
   * @brief Get the current control command.
   *
   * This method should return the last control command computed.
   * It should not run the control algorithm (see update method).
   *
   * @return A TwistStamped message with the current control command.
   */
  [[nodiscard]] virtual geometry_msgs::msg::TwistStamped get_cmd_vel() = 0;

  /**
   * @brief Run the control method and update the control command.
   *
   * This method will be called by the system's ControllerNode to run the control algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState nav_state) = 0;

private:
  /**
   * @brief Pointer to the parent lifecycle node.
   *
   * This is used to access ROS interfaces and parameters.
   */
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};
};

}  // namespace easynav

#endif  // EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
