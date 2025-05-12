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
/// \brief A blackboard-like structure to hold the current state of the navigation system.

#ifndef EASYNAV_COMMON_TYPES__NAVSTATE_HPP_
#define EASYNAV_COMMON_TYPES__NAVSTATE_HPP_

#include "rclcpp/time.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/goals.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/MapTypeBase.hpp"

namespace easynav
{

/**
 * @brief Represents the state of the navigation system.
 *
 * This structure contains information about the current position, velocity,
 * and other relevant data for the navigation system.
 */
struct NavState
{
  /**
   * @brief The timestamp of the current navigation state.
   */
  rclcpp::Time timestamp;

  /**
   * @brief The current position of the robot in global coordinates.
   */
  nav_msgs::msg::Odometry odom;

  /**
  * @brief The current perception data.
  */
  Perceptions perceptions;

  /**
  * @brief The current list of map representations.
  */
  std::map<std::string, std::shared_ptr<MapsTypeBase>> maps;

  /**
  * @brief The current path.
  */
  nav_msgs::msg::Path path;

  /**
  * @brief The current goal (list of goals).
  */
  nav_msgs::msg::Goals goals;

  /**
   * @brief The current velocity command.
   */
  geometry_msgs::msg::TwistStamped cmd_vel;
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__NAVSTATE_HPP_
