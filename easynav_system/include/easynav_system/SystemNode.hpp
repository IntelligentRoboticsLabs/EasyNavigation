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
/// \brief Declaration of the SystemNode class, the central coordinator node for Easy Navigation components.

#ifndef EASYNAV_SYSTEM__SYSTEMNODE_HPP_
#define EASYNAV_SYSTEM__SYSTEMNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"

namespace easynav
{

/**
 * @struct SystemNodeInfo
 * @brief Structure holding pointers to runtime info of each node.
 */
struct SystemNodeInfo
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr; ///< Shared pointer to the managed lifecycle node.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg;       ///< Associated real-time callback group.
};

/**
 * @class SystemNode
 * @brief ROS 2 lifecycle node that coordinates all major modules of the Easy Navigation system.
 *
 * This node manages the lifecycle, scheduling, and interactions between the Controller, Localizer,
 * Planner, Maps Manager, and Sensors modules, providing a unified interface to ROS 2.
 */
class SystemNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SystemNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a SystemNode lifecycle node with the specified options.
   * @param options Node options to configure the SystemNode node.
   */
  explicit SystemNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the SystemNode object.
   */
  ~SystemNode();

  /**
   * @brief Configures the SystemNode node.
   * Declares parameters and initializes submodules (maps manager, planner, etc.).
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the SystemNode node.
   * Starts periodic cycles and activates all submodules.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the SystemNode node.
   * Stops periodic tasks and deactivates all submodules.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the SystemNode node.
   * Resets submodules and releases allocated resources.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the SystemNode node.
   * Performs final shutdown procedures.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the SystemNode node.
   * Called when an error occurs during a lifecycle transition.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * Callbacks requiring low latency or real-time constraints should use this group.
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Retrieves references to all system nodes managed by SystemNode.
   *
   * @return A map associating node names with their node pointers and callback groups.
   */
  std::map<std::string, SystemNodeInfo> get_system_nodes();

private:
  /**
   * @brief Callback group intended for real-time system operations.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the real-time system cycle.
   */
  rclcpp::TimerBase::SharedPtr system_main_rt_timer_;

  /**
   * @brief Timer that triggers the non-real-time system cycle.
   */
  rclcpp::TimerBase::SharedPtr system_main_nort_timer_;

  /**
   * @brief Controller module node.
   */
  ControllerNode::SharedPtr controller_node_;

  /**
   * @brief Localizer module node.
   */
  LocalizerNode::SharedPtr localizer_node_;

  /**
   * @brief Maps Manager module node.
   */
  MapsManagerNode::SharedPtr maps_manager_node_;

  /**
   * @brief Planner module node.
   */
  PlannerNode::SharedPtr planner_node_;

  /**
   * @brief Sensors module node.
   */
  SensorsNode::SharedPtr sensors_node_;

  /**
   * @brief The current navigation state.
   */
  NavState nav_state_;

  /**
   * @brief Executes one cycle of real-time system operations.
   *
   * This function is called periodically by the real-time timer to manage control,
   * localization, planning, and other tightly coupled tasks.
   */
  void system_cycle_rt();

  /**
   * @brief Executes one cycle of non-real-time system operations.
   *
   * This function manages background tasks not requiring strict real-time execution.
   */
  void system_cycle_nort();
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__SYSTEMNODE_HPP_
