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
/// \brief Implementation of the GoalManager class.

#include "easynav_system/GoalManager.hpp"

namespace easynav
{

std::expected<void, std::string>
GoalManager::initialize(rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
{
  parent_node_ = parent_node;

  goal_sub_ = parent_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal", 10,
    std::bind(&GoalManager::goal_callback, this, std::placeholders::_1)
  );

  cancel_goal_sub_ = parent_node_->create_subscription<std_msgs::msg::Empty::SharedPtr>(
    "cancel_goal", 10,
    std::bind(&GoalManager::cancel_goal_callback, this, std::placeholders::_1)
  );

  pause_goal_sub_ = parent_node_->create_subscription<std_msgs::msg::Empty::SharedPtr>(
    "pause_goal", 10,
    std::bind(&GoalManager::pause_goal_callback, this, std::placeholders::_1)
  );

  status_pub_ = parent_node_->create_publisher<easynav_interfaces::msg::NavigationStatus>(
    "nav_feedback", 10);

  return {};
}

void GoalManager::goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  goal_ = *msg;
  // TODO: Handle (new?) goal
}

void GoalManager::cancel_goal_callback(const std_msgs::msg::Empty::SharedPtr)
{
  // TODO: Handle goal cancellation
}

void GoalManager::pause_goal_callback(const std_msgs::msg::Empty::SharedPtr)
{
  // TODO: Handle goal pause
}


}  // namespace easynav
