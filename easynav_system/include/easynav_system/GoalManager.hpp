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

#include "easynav_interfaces/msg/navigation_control.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/goals.hpp"

#include "easynav_common/types/NavState.hpp"

namespace easynav
{

class GoalManager
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GoalManager)

  GoalManager(
    const std::shared_ptr<const NavState> & nav_state,
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  [[nodiscard]] inline nav_msgs::msg::Goals get_goals() const {return goals_;}
  [[nodiscard]] inline int get_state() const {return status_;}

  void update();

  static const int IDLE = 0;
  static const int ACTIVE = 1;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;
  nav_msgs::msg::Goals goals_;

  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr comanded_pose_sub_;

  easynav_interfaces::msg::NavigationControl::UniquePtr last_control_;

  std::string id_;
  bool allow_preempt_goal_ {true};
  rclcpp::Time nav_start_time_;
  /**
   * @brief The current navigation state.
   */
  const std::shared_ptr<const NavState> nav_state_;

  void control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg);
  void comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

  int status_ {IDLE};
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGER_HPP_
