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
/// \brief Declaration of the PlannerNode class, a ROS 2 lifecycle node for computing navigation paths in Easy Navigation.

#ifndef EASYNAV_PLANNER__PLANNERNODE_HPP_
#define EASYNAV_PLANNER__PLANNERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_core/PlannerMethodBase.hpp"
#include "pluginlib/class_loader.hpp"
#include "nav_msgs/msg/path.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class PlannerNode
 * @brief ROS 2 lifecycle node that manages path planning in Easy Navigation.
 *
 * Handles lifecycle transitions, plugin loading, and invokes the planner to compute paths.
 */
class PlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PlannerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param nav_state Shared pointer to the navigation state.
   * @param options Optional node configuration.
   */
  explicit PlannerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~PlannerNode();

  /**
   * @brief Configure the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Clean up the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition errors.
   * @param state Current lifecycle state.
   * @return SUCCESS if error handling completed.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the current planned path.
   * @return Path message with the last computed path.
   */
  [[nodiscard]] nav_msgs::msg::Path get_path() const;

  /**
   * @brief Execute a non-real-time cycle.
   */
  void cycle();

private:
  /// @brief Shared navigation state.
  const std::shared_ptr<const NavState> nav_state_;

  /// @brief Plugin loader for planner methods.
  std::unique_ptr<pluginlib::ClassLoader<PlannerMethodBase>> planner_loader_;

  /// @brief Loaded planner plugin.
  std::shared_ptr<PlannerMethodBase> planner_method_ {nullptr};
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__PLANNERNODE_HPP_
