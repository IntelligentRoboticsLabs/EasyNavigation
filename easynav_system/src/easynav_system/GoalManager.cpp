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

GoalManager::GoalManager(
  const std::shared_ptr<const NavState> & nav_state,
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
: parent_node_(parent_node),
  nav_state_(nav_state)
{
  status_ = IDLE;

  parent_node_->declare_parameter("allow_preempt_goal", allow_preempt_goal_);
  parent_node_->get_parameter("allow_preempt_goal", allow_preempt_goal_);

  control_sub_ = parent_node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
    std::string(parent_node_->get_name()) + "control", 100,
    std::bind(&GoalManager::control_callback, this, std::placeholders::_1)
  );

  comanded_pose_sub_ = parent_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100,
    std::bind(&GoalManager::comanded_pose_callback, this, std::placeholders::_1)
  );

  control_pub_ = parent_node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
    std::string(parent_node_->get_name()) + "control", 100);

  id_ = "easynav_system";
}

void
GoalManager::control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  if (msg->user_id == id_) {return;}  // Avoid self messages

  easynav_interfaces::msg::NavigationControl response;
  response = *msg;
  response.header.stamp = parent_node_->now();
  response.seq = msg->seq + 1;
  response.user_id = id_;

  switch (msg->status) {
    case easynav_interfaces::msg::NavigationControl::REQUEST:
      if (status_ == IDLE || allow_preempt_goal_) {
        if (!msg->goals.goals.empty()) {
          nav_start_time_ = parent_node_->now();

          goals_ = msg->goals;
          response.status_message = "Goal accepted";
          response.status = easynav_interfaces::msg::NavigationControl::ACCEPT;
        } else {
          response.status_message = "Goals are empty";
          response.status = easynav_interfaces::msg::NavigationControl::REJECT;
        }
      } else {
        response.status_message = "Goal rejected; unable to preemp current active goal";
        response.status = easynav_interfaces::msg::NavigationControl::REJECT;
      }
      break;
    case easynav_interfaces::msg::NavigationControl::CANCEL:
      if (status_ == IDLE) {
        response.status_message = "Nothing to cancel; easynav is idle";
        response.status = easynav_interfaces::msg::NavigationControl::ERROR;
      } else {
        status_ = IDLE;
        response.status_message = "Goal cancelled";
        response.status = easynav_interfaces::msg::NavigationControl::CANCELLED;
      }
      break;
    default:
      RCLCPP_WARN(parent_node_->get_logger(), "Received erroneous control message %d", msg->status);
      response.status_message = "Unable to process message";
      response.status = easynav_interfaces::msg::NavigationControl::ERROR;
      break;
  }

  control_pub_->publish(response);
  last_control_ = std::move(msg);
}

void
GoalManager::comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  easynav_interfaces::msg::NavigationControl command;
  command.header.stamp = parent_node_->now();
  command.seq = last_control_->seq + 1;
  command.user_id = id_ + "_command";
  command.status = easynav_interfaces::msg::NavigationControl::REQUEST;
  command.goals.goals.push_back(*msg);

  control_pub_->publish(command);
  *last_control_ = command;
}

void
GoalManager::update()
{
  if (status_ == IDLE) {return;}

  easynav_interfaces::msg::NavigationControl feedback;
  feedback.header.stamp = parent_node_->now();
  feedback.seq = last_control_->seq + 1;
  feedback.user_id = id_;

  feedback.goals = nav_state_->goals;
  feedback.current_pose.header = nav_state_->odom.header;
  feedback.current_pose.pose = nav_state_->odom.pose.pose;
  feedback.navigation_time = parent_node_->now() - nav_start_time_;

  // ToDo[@fmrico]: Complete feedback info

  control_pub_->publish(feedback);
  *last_control_ = feedback;
}

}  // namespace easynav
