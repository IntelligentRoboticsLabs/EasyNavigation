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
/// \brief Declaration of the abstract base class LocalizerMethodBase.

#ifndef EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
#define EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class LocalizerMethodBase
 * @brief Abstract base class for localization methods in Easy Navigation.
 *
 * This class serves as a template for implementing various localization
 * algorithms in the Easy Navigation framework.
 *
 * The actual localization method should be implemented by extending this base class
 * and registering the derived class as a plugin with pluginlib,
 * which will be loaded at runtime in the system.
 */
class LocalizerMethodBase
{
public:
  /**
   * @brief Default constructor for LocalizerMethodBase.
   */
  LocalizerMethodBase() = default;

  /**
   * @brief Virtual destructor for LocalizerMethodBase.
   *
   * This ensures proper cleanup of derived classes.
   */
  virtual ~LocalizerMethodBase() = default;

  /**
   * @brief Initialize the localization method.
   *
   * This method should be called to set up any necessary resources
   * or configurations for the localization algorithm.
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
   * @brief Get the current localization state.
   *
   * This method should return the last known localization of the robot.
   * It should not run the localization algorithm (see update method).
   *
   * @return An Odometry message representing the current localization state.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() = 0;

  /**
   * @brief Run the localization method and update the robot's estimated localization.
   *
   * This method will be called by the system's LocalizerNode to run the localization algorithm.
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
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
