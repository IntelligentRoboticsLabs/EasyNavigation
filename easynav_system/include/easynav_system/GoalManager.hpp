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
/// \brief Declaration of the GoalManager class.

#ifndef EASYNAV_SYSTEM__GOALMANAGER_HPP_
#define EASYNAV_SYSTEM__GOALMANAGER_HPP_

#include <expected>

#include "rclcpp/subscription.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_interfaces/msg/navigation_status.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/empty.hpp"

namespace easynav
{

class GoalManager
{
public:
  using NavigationStatus = easynav_interfaces::msg::NavigationStatus;

  RCLCPP_SMART_PTR_DEFINITIONS(GoalManager)

  GoalManager() = default;

  std::expected<void, std::string>
  initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  [[nodiscard]] inline geometry_msgs::msg::PoseStamped get_goal() const {return goal_;}

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;
  geometry_msgs::msg::PoseStamped goal_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty::SharedPtr>::SharedPtr cancel_goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty::SharedPtr>::SharedPtr pause_goal_sub_;

  rclcpp::Publisher<NavigationStatus>::SharedPtr status_pub_;
  NavigationStatus nav_status_;

  void goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void cancel_goal_callback(const std_msgs::msg::Empty::SharedPtr msg);
  void pause_goal_callback(const std_msgs::msg::Empty::SharedPtr msg);
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGER_HPP_
