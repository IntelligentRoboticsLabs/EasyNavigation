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
  state_ = State::IDLE;

  parent_node_->declare_parameter("allow_preempt_goal", allow_preempt_goal_);
  parent_node_->get_parameter("allow_preempt_goal", allow_preempt_goal_);

  control_sub_ = parent_node_->create_subscription<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100,
    std::bind(&GoalManager::control_callback, this, std::placeholders::_1)
  );

  comanded_pose_sub_ = parent_node_->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 100,
    std::bind(&GoalManager::comanded_pose_callback, this, std::placeholders::_1)
  );

  control_pub_ = parent_node_->create_publisher<easynav_interfaces::msg::NavigationControl>(
    "easynav_control", 100);

  id_ = "easynav_system";
  last_control_ = std::make_unique<easynav_interfaces::msg::NavigationControl>();
}

void
GoalManager::accept_request(
  const easynav_interfaces::msg::NavigationControl & msg,
  easynav_interfaces::msg::NavigationControl & response)
{
  nav_start_time_ = parent_node_->now();

  RCLCPP_DEBUG(parent_node_->get_logger(), "Accepted navigation request");

  goals_ = msg.goals;

  current_client_id_ = msg.user_id;
  response.status_message = "Goal accepted";
  response.type = easynav_interfaces::msg::NavigationControl::ACCEPT;
  response.nav_current_user_id = current_client_id_;
  state_ = State::ACTIVE;
}


void
GoalManager::control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg)
{
  if (msg->user_id == id_) {return;}  // Avoid self messages

  RCLCPP_DEBUG(parent_node_->get_logger(), "Managing navigation control message received");

  easynav_interfaces::msg::NavigationControl response;
  response = *msg;
  response.header.stamp = parent_node_->now();
  response.seq = msg->seq + 1;
  response.user_id = id_;

  switch (msg->type) {
    case easynav_interfaces::msg::NavigationControl::REQUEST:
      RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation request");
      if (msg->goals.goals.empty()) {
        RCLCPP_DEBUG(parent_node_->get_logger(), "Rejected navigation request (empty goals)");

        response.status_message = "Goals are empty";
        response.type = easynav_interfaces::msg::NavigationControl::REJECT;
        response.nav_current_user_id = msg->user_id;
      } else {
        if (state_ == State::IDLE) {
          accept_request(*msg, response);
        } else {
          if (allow_preempt_goal_) {
            if (msg->user_id != current_client_id_) {
              set_preempted();
            }
            accept_request(*msg, response);
          } else {
            RCLCPP_DEBUG(parent_node_->get_logger(),
              "Rejected navigation request (unable to preempt)");

            response.status_message = "Goal rejected; unable to preemp current active goal";
            response.type = easynav_interfaces::msg::NavigationControl::REJECT;
            response.nav_current_user_id = msg->user_id;
          }
        }
      }
      break;
    case easynav_interfaces::msg::NavigationControl::CANCEL:
      RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation requested");

      if (current_client_id_ != msg->user_id) {
        RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation rejected (not yours)");
        response.status_message = "Trying to cancel a navigation not commanded by you";
        response.type = easynav_interfaces::msg::NavigationControl::REJECT;
        response.nav_current_user_id = msg->user_id;
      } else {
        if (state_ == State::IDLE) {
          RCLCPP_DEBUG(parent_node_->get_logger(),
            "Navigation cancelation rejected (not navigating)");
          response.status_message = "Nothing to cancel; easynav is idle";
          response.type = easynav_interfaces::msg::NavigationControl::ERROR;
          response.nav_current_user_id = msg->user_id;
        } else {
          RCLCPP_DEBUG(parent_node_->get_logger(), "Navigation cancelation accepted");
          state_ = State::IDLE;
          goals_.goals.clear();
          response.status_message = "Goal cancelled";
          response.type = easynav_interfaces::msg::NavigationControl::CANCELLED;
          response.nav_current_user_id = current_client_id_;
          state_ = State::IDLE;
        }
      }
      break;
    default:
      RCLCPP_WARN(parent_node_->get_logger(), "Received erroneous control message %d", msg->type);
      response.status_message = "Unable to process message";
      response.type = easynav_interfaces::msg::NavigationControl::ERROR;
      response.nav_current_user_id = msg->user_id;
      break;
  }

  control_pub_->publish(response);
  last_control_ = std::move(msg);
}

void
GoalManager::set_preempted()
{
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::CANCELLED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = "Navigation preempted by others";

  control_pub_->publish(response);
}

void
GoalManager::set_finished()
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::FINISHED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = "Navigation succesfully finished";

  control_pub_->publish(response);
}

void
GoalManager::set_error(const std::string & reason)
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::ERROR;
  response.nav_current_user_id = current_client_id_;
  response.status_message = reason;

  control_pub_->publish(response);
}

void
GoalManager::set_failed(const std::string & reason)
{
  state_ = State::IDLE;
  goals_.goals.clear();

  easynav_interfaces::msg::NavigationControl response;
  response = *last_control_;
  response.header.stamp = parent_node_->now();
  response.seq = last_control_->seq + 1;
  response.user_id = id_;
  response.type = easynav_interfaces::msg::NavigationControl::FAILED;
  response.nav_current_user_id = current_client_id_;
  response.status_message = reason;

  control_pub_->publish(response);
}

void
GoalManager::comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg)
{
  auto command = std::make_unique<easynav_interfaces::msg::NavigationControl>();
  command->header = msg->header;
  command->seq = last_control_->seq + 1;
  command->user_id = id_ + "_initpose";
  command->type = easynav_interfaces::msg::NavigationControl::REQUEST;
  command->goals.header = msg->header;
  command->goals.goals.push_back(*msg);

  control_callback(std::move(command));
}

void
GoalManager::update()
{
  if (state_ == State::IDLE) {return;}

  easynav_interfaces::msg::NavigationControl feedback;
  feedback.type = easynav_interfaces::msg::NavigationControl::FEEDBACK;
  feedback.header.stamp = parent_node_->now();
  feedback.seq = last_control_->seq + 1;
  feedback.user_id = id_;
  feedback.nav_current_user_id = current_client_id_;

  feedback.goals = nav_state_->goals;
  feedback.current_pose.header = nav_state_->odom.header;
  feedback.current_pose.pose = nav_state_->odom.pose.pose;
  feedback.navigation_time = parent_node_->now() - nav_start_time_;

  // ToDo[@fmrico]: Complete feedback info

  RCLCPP_DEBUG(parent_node_->get_logger(), "Sending navigation feedback");
  control_pub_->publish(feedback);
  *last_control_ = feedback;
}

}  // namespace easynav
