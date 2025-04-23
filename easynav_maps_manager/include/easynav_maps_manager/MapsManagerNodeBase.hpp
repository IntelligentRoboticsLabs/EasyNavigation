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
/// \brief Declaration of the MapsManagerNodeBase lifecycle node, ROS 2 interface for EasyNav core.

#ifndef EASYNAV_MAPSMANAGER__MAPS_MANAGER_NODE_BASE_HPP_
#define EASYNAV_MAPSMANAGER__MAPS_MANAGER_NODE_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_maps_manager/MapTypeBase.hpp"

namespace easynav_maps_manager
{

/// \file
/// \brief Declaration of the MapsManagerNodeBase class, a ROS 2 lifecycle node for map handling in Easy Navigation.

/**
 * @class MapsManagerNodeBase
 * @brief ROS 2 lifecycle node that manages mapping-related tasks for the Easy Navigation system.
 *
 * This node is responsible for orchestrating the mapping functionality in the EasyNav architecture.
 * It includes lifecycle management, real-time callback group assignment, and periodic execution of
 * map-related operations such as updates, data fusion, and diagnostics.
 */

class MapsManagerNodeBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  // RCLCPP_SMART_PTR_DEFINITIONS(MapsManagerNodeBase)  // Not possible with abstract classes
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a MapsManagerNodeBase lifecycle node with the specified options.
   * @param options Node options to configure the MapsManagerNodeBase node.
   */
  explicit MapsManagerNodeBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configures the MapsManagerNodeBase node.
   * This is typically where parameters and interfaces are declared.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the MapsManagerNodeBase node.
   * This starts periodic navigation control cycles.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the MapsManagerNodeBase node.
   * Control loops are stopped and interfaces are disabled.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the MapsManagerNodeBase node.
   * Releases resources and resets the internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the MapsManagerNodeBase node.
   * Called on final shutdown of the node's lifecycle.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the MapsManagerNodeBase node.
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


  /**
   * @brief Returns the last robot pose estimated by the Localizer
   *
   * @return An Odometry object representing the robot state in the global frame
   */
  std::shared_ptr<const MapsTypeBase> get_map() const;

protected:
  /**
   * @brief Executes a single cycle. Must be implemented by derived classes
   *
   * This method is periodically called by a timer to run the maps_manager logic
   */
  virtual void maps_manager_cycle() = 0;

  /**
   * @brief The environment representation, as a pointer to the base type MapsTypeBase
   * TODO: Handle both the static and the dynamic map
   */
  std::shared_ptr<MapsTypeBase> map_ {};

private:
  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic map tasks cycle.
   */
  rclcpp::TimerBase::SharedPtr maps_manager_main_timer_;
};

/**
 * @class SimpleMapsManager
 * @brief This is only an example class that implements
 *        the interface defined in MapsManagerNodeBase
 *
 */
class SimpleMapsManager : public MapsManagerNodeBase
{
protected:
  /**
  * @brief Executes a single cycle. Must be implemented by derived classes.
  *
  * This method should compute the actual pose of the robot and store it in robot_odom_
  */
  virtual void maps_manager_cycle() override
  {
    // Example implementation for the control cycle
    // map_->update_dynamic_map(perceptions);
    map_->publish_static_map();
    map_->publish_dynamic_map();
  }
};

}  // namespace easynav_maps_manager

#endif  // EASYNAV_MAPSMANAGER__MAPS_MANAGER_NODE_BASE_HPP_
