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
/// \brief Declaration of the PlannerNode lifecycle node, ROS 2 interface for EasyNav core.

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

/// \file
/// \brief Declaration of the PlannerNode class, a ROS 2 lifecycle node for computing navigation paths in Easy Navigation.

/**
 * @class PlannerNode
 * @brief ROS 2 lifecycle node that manages path planning for the Easy Navigation system.
 *
 * This node provides the interface between the planner plugin and the ROS 2 ecosystem.
 * It manages lifecycle transitions, real-time callback scheduling, and invokes the underlying
 * PlannerMethodBase implementation to generate navigation paths.
 */
class PlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(PlannerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a PlannerNode lifecycle node with the specified options.
   * @param options Node options to configure the PlannerNode node.
   */
  explicit PlannerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the PlannerNode object.
   */
  ~PlannerNode();

  /**
   * @brief Configures the PlannerNode node.
   * This is typically where parameters and interfaces are declared.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the PlannerNode node.
   * This starts periodic planning updates.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the PlannerNode node.
   * Control loops are stopped and interfaces are disabled.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the PlannerNode node.
   * Releases resources and resets internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the PlannerNode node.
   * Called during the final shutdown phase.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors during lifecycle transitions.
   *
   * This method is triggered if an error occurs while transitioning states.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the most recent computed navigation path.
   *
   * @return nav_msgs::msg::Path representing the current navigation plan.
   */
  [[nodiscard]] nav_msgs::msg::Path get_path() const;

  /**
   * @brief Non-real-time cycle execution.
   *
   * Used for lower-priority updates or background processing.
   */
  void cycle();

private:
  /**
   * @brief Pointer to the loaded planner method plugin.
   *
   * This is the concrete implementation of the planning algorithm.
   */
  std::shared_ptr<PlannerMethodBase> planner_method_ {nullptr};

  /**
   * @brief The current navigation state.
   */
  const std::shared_ptr<const NavState> nav_state_;

  /**
   * @brief Plugin loader for planner methods.
   *
   * This loads the user-specified planning algorithm from a plugin.
   */
  std::unique_ptr<pluginlib::ClassLoader<easynav::PlannerMethodBase>> planner_loader_;
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__PLANNERNODE_HPP_
