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
/// \brief Declaration of the GoalManagerClient class.

#ifndef EASYNAV_SYSTEM__GOALMANAGERCLIENT_HPP_
#define EASYNAV_SYSTEM__GOALMANAGERCLIENT_HPP_


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_interfaces/msg/navigation_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"


namespace easynav
{

using namespace std::placeholders;

class GoalManagerClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GoalManagerClient)

  enum class State
  {
    IDLE,
    SENT_GOAL,
    SENT_PREEMPT,
    ACCEPTED_AND_NAVIGATING,
    NAVIGATION_FINISHED,
    NAVIGATION_REJECTED,
    NAVIGATION_FAILED,
    NAVIGATION_CANCELLED,
    ERROR
  };

  GoalManagerClient(rclcpp::Node::SharedPtr node);

  void send_goal(const geometry_msgs::msg::PoseStamped & goal);
  void send_goals(const nav_msgs::msg::Goals & goals);
  void cancel();
  void reset();

  [[nodiscard]] State get_state() const {return state_;}
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_last_control() const;
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_feedback() const;
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_result() const;

private:
  rclcpp::Node::SharedPtr node_;

  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;

  easynav_interfaces::msg::NavigationControl::UniquePtr last_control_;
  easynav_interfaces::msg::NavigationControl last_feedback_;
  easynav_interfaces::msg::NavigationControl last_result_;

  std::string id_;
  std::string current_client_id_;
  State state_;

  void control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg);
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGERCLIENT_HPP_
