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
#include "easynav_core/MapsManagerBase.hpp"
#include "pluginlib/class_loader.hpp"

namespace easynav
{

/**
 * @class MapsManagerNode
 * @brief ROS 2 lifecycle node that manages mapping-related tasks for the Easy Navigation system.
 *
 * This node acts as the orchestrator for multiple map manager plugins, each responsible
 * for a specific type of map representation. It handles lifecycle state transitions,
 * loads plugins dynamically, and schedules map updates in real-time or background cycles.
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
  explicit MapsManagerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destructor.
   */
  ~MapsManagerNode();

  /**
   * @brief Configures the MapsManagerNode node.
   *
   * Loads parameters, instantiates maps manager plugins, and prepares internal structures.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the MapsManagerNode node.
   *
   * Starts timers and enables execution of map-related tasks.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the MapsManagerNode node.
   *
   * Stops timers and disables map-related interfaces.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the MapsManagerNode node.
   *
   * Releases all resources and resets the internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the MapsManagerNode node.
   *
   * Called at final shutdown of the lifecycle node.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the MapsManagerNode node.
   *
   * Invoked when a transition fails and recovery or diagnostics are needed.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * This callback group is used for low-latency tasks that must not block.
   *
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Executes one cycle of real-time localization tasks.
   *
   * Typically called from a real-time timer. Invokes the plugin's update method.
   */
  bool maps_manager_cycle_rt(bool trigger = false);

private:
  /**
   * @brief Callback group intended for real-time map operations.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic map update cycle.
   */
  rclcpp::TimerBase::SharedPtr maps_manager_main_timer_;

  /**
   * @brief List of active map instances in memory.
   */
  std::vector<std::shared_ptr<MapsTypeBase>> maps_;

  /**
   * @brief Executes one cycle of non-real-time operations.
   *
   * Reserved for diagnostics, visualization, or heavy I/O.
   */
  void maps_manager_cycle_nort();

  /**
   * @brief Plugin loader for MapsManagerBase-based implementations.
   */
  std::unique_ptr<pluginlib::ClassLoader<MapsManagerBase>> maps_manager_loader_;

  /**
   * @brief List of active map manager plugin instances.
   */
  std::vector<std::shared_ptr<MapsManagerBase>> maps_managers_;

  /**
   * @brief Latest known navigation state.
   */
  const std::shared_ptr<const NavState> nav_state_;
};

}  // namespace easynav

#endif  // EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
