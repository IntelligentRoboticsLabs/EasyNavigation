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
/// \brief Declaration of the MapsManagerNode class, a ROS 2 lifecycle node for map handling in Easy Navigation.

#ifndef EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
#define EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_common/types/MapTypeBase.hpp"

namespace easynav
{

/**
 * @class MapsManagerNode
 * @brief ROS 2 lifecycle node that manages mapping-related tasks for the Easy Navigation system.
 *
 * This node is responsible for orchestrating the mapping functionality in the EasyNav architecture.
 * It includes lifecycle management, real-time callback group assignment, and periodic execution of
 * map-related operations such as updates, data fusion, and diagnostics.
 */

class MapsManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MapsManagerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a MapsManagerNode lifecycle node with the specified options.
   * @param options Node options to configure the MapsManagerNode node.
   */
  explicit MapsManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~MapsManagerNode();

  /**
   * @brief Configures the MapsManagerNode node.
   * This is typically where parameters and interfaces are declared.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the MapsManagerNode node.
   * This starts periodic navigation control cycles.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the MapsManagerNode node.
   * Control loops are stopped and interfaces are disabled.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the MapsManagerNode node.
   * Releases resources and resets the internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the MapsManagerNode node.
   * Called on final shutdown of the node's lifecycle.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the MapsManagerNode node.
   * This is called when a failure occurs during a lifecycle transition.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * This callback group can be used to assign callbacks that require
   * low latency or have real-time constraints.
   *
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

private:
  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic map tasks cycle.
   */
  rclcpp::TimerBase::SharedPtr maps_manager_main_timer_;

  /** List of map representations */
  std::vector<std::shared_ptr<MapsTypeBase>> maps_;
  /**
   * @brief Executes one cycle of real-time system operations.
   *
   * This function is called periodically by the real-time timer to manage control,
   * localization, planning, and other tightly coupled tasks.
   */
  void maps_manager_cycle_rt();

  /**
   * @brief Executes one cycle of non-real-time system operations.
   *
   * This function manages background tasks not requiring strict real-time execution.
   */
  void maps_manager_cycle_nort();
};

}  // namespace easynav

#endif  // EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
