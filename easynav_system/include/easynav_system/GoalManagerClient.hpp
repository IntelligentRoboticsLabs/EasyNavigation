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

/**
 * @class GoalManagerClient
 * @brief Client-side interface for interacting with GoalManager.
 *
 * Sends navigation goals, handles feedback and result updates, and manages goal lifecycle state.
 */
class GoalManagerClient
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GoalManagerClient)

  /**
   * @enum State
   * @brief Internal state of the client-side goal manager.
   */
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

  /**
   * @brief Constructor.
   * @param node Shared pointer to a ROS 2 node.
   */
  GoalManagerClient(rclcpp::Node::SharedPtr node);

  /**
   * @brief Send a single goal to the GoalManager.
   * @param goal PoseStamped representing the goal.
   */
  void send_goal(const geometry_msgs::msg::PoseStamped & goal);

  /**
   * @brief Send a list of goals to the GoalManager.
   * @param goals List of goals to navigate.
   */
  void send_goals(const nav_msgs::msg::Goals & goals);

  /**
   * @brief Cancel the current goal.
   */
  void cancel();

  /**
   * @brief Reset internal client state.
   */
  void reset();

  /**
   * @brief Get the current internal state.
   * @return Current State.
   */
  [[nodiscard]] State get_state() const {return state_;}

  /**
   * @brief Get the last control message sent or received.
   * @return Reference to the last control message.
   */
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_last_control() const;

  /**
   * @brief Get the most recent feedback received.
   * @return Reference to the last feedback message.
   */
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_feedback() const;

  /**
   * @brief Get the last result message received.
   * @return Reference to the last result message.
   */
  [[nodiscard]] const easynav_interfaces::msg::NavigationControl & get_result() const;

private:
  /// @brief Underlying ROS 2 node.
  rclcpp::Node::SharedPtr node_;

  /// @brief Publisher for control commands.
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;

  /// @brief Subscriber for control responses, feedback, and results.
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;

  /// @brief Last received control message (raw pointer).
  easynav_interfaces::msg::NavigationControl::UniquePtr last_control_;

  /// @brief Last feedback message.
  easynav_interfaces::msg::NavigationControl last_feedback_;

  /// @brief Last result message.
  easynav_interfaces::msg::NavigationControl last_result_;

  /// @brief My ID.
  std::string id_;

  /// @brief ID of the client sending goals.
  std::string current_client_id_;

  /// @brief Internal state of the client.
  State state_;

  /// @brief Callback for incoming control messages.
  void control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg);
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGERCLIENT_HPP_
