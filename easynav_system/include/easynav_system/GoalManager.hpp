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

/**
 * @class GoalManager
 * @brief Handles navigation goals, their lifecycle, and command interface.
 *
 * Manages goal state, handles external control messages, and interacts with navigation components.
 */
class GoalManager
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(GoalManager)

  /**
   * @enum State
   * @brief Current internal goal state.
   */
  enum class State
  {
    IDLE,   ///< No active goal.
    ACTIVE  ///< A goal is currently being pursued.
  };

  /**
   * @brief Constructor.
   * @param nav_state Shared pointer to navigation state.
   * @param parent_node Lifecycle node for parameter and interface management.
   */
  GoalManager(
    const std::shared_ptr<const NavState> & nav_state,
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node);

  /**
   * @brief Get current goals.
   * @return Goals message.
   */
  [[nodiscard]] inline nav_msgs::msg::Goals get_goals() const {return goals_;}

  /**
   * @brief Get current internal goal state.
   * @return GoalManager::State value.
   */
  [[nodiscard]] inline State get_state() const {return state_;}

  /**
   * @brief Mark the current goal as successfully completed.
   */
  void set_finished();

  /**
   * @brief Mark the current goal as failed.
   * @param reason Textual explanation.
   */
  void set_failed(const std::string & reason);

  /**
   * @brief Mark the system as in error state.
   * @param reason Textual explanation.
   */
  void set_error(const std::string & reason);

  /**
   * @brief Update internal logic, including preemption and timeout checks.
   */
  void update();

  void check_goals(
    const geometry_msgs::msg::Pose & current_pose,
    double position_tolerance, double angle_tolerance);

private:
  /// @brief Lifecycle node.
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_;

  /// @brief Currently active goals.
  nav_msgs::msg::Goals goals_;

  /// @brief Publisher for goal control responses.
  rclcpp::Publisher<easynav_interfaces::msg::NavigationControl>::SharedPtr control_pub_;

  /// @brief Subscription to external goal control commands.
  rclcpp::Subscription<easynav_interfaces::msg::NavigationControl>::SharedPtr control_sub_;

  /// @brief Subscription to pose-stamped goals (GUI or RViz).
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr comanded_pose_sub_;

  /// @brief Last received navigation control message.
  easynav_interfaces::msg::NavigationControl::UniquePtr last_control_;

  /// @brief My ID.
  std::string id_;

  /// @brief ID of the client who sent the current goal.
  std::string current_client_id_;

  /// @brief Whether goal preemption is allowed.
  bool allow_preempt_goal_ {true};

  /// @brief Timestamp when the current navigation started.
  rclcpp::Time nav_start_time_;

  /// @brief Shared pointer to current navigation state.
  const std::shared_ptr<const NavState> nav_state_;

  /// @brief Handle new goal request and populate the response.
  void accept_request(
    const easynav_interfaces::msg::NavigationControl & msg,
    easynav_interfaces::msg::NavigationControl & response);

  /// @brief Handle incoming control messages.
  void control_callback(easynav_interfaces::msg::NavigationControl::UniquePtr msg);

  /// @brief Handle goal poses received via PoseStamped messages.
  void comanded_pose_callback(geometry_msgs::msg::PoseStamped::UniquePtr msg);

  /// @brief Mark current goal as preempted.
  void set_preempted();

  /// @brief Internal goal state.
  State state_;
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__GOALMANAGER_HPP_
