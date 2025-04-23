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
/// \brief Declaration of the SensorsNodeBase lifecycle node, ROS 2 interface for EasyNav core.

#ifndef EASYNAV_SENSORS__SENSORS_NODE_BASE_HPP_
#define EASYNAV_SENSORS__SENSORS_NODE_BASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_sensors/Perceptions.hpp"

namespace easynav_sensors
{

/// \file
/// \brief Declaration of the SensorsNodeBase class, a ROS 2 lifecycle node for sensor fussion tasks in Easy Navigation.

/**
 * @class SensorsNodeBase
 * @brief ROS 2 lifecycle node that manages sensors for the Easy Navigation system.
 *
 * This node provides the interface between the sensor module in EasyNav and the ROS 2 ecosystem.
 * It handles lifecycle transitions, real-time scheduling of periodic tasks, and parameter setup.
 */

class SensorsNodeBase : public rclcpp_lifecycle::LifecycleNode
{
public:
  // RCLCPP_SMART_PTR_DEFINITIONS(LocalizerNodeBase)  // Not possible with abstract classes
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a SensorsNodeBase lifecycle node with the specified options.
   * @param options Node options to configure the SensorsNodeBase node.
   */
  explicit SensorsNodeBase(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configures the SensorsNodeBase node.
   * This is typically where parameters and interfaces are declared.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the SensorsNodeBase node.
   * This starts periodic navigation control cycles.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the SensorsNodeBase node.
   * Control loops are stopped and interfaces are disabled.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the SensorsNodeBase node.
   * Releases resources and resets the internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the SensorsNodeBase node.
   * Called on final shutdown of the node's lifecycle.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the SensorsNodeBase node.
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
   * @brief Returns the current list of perceptions
   *
   * @return A Perceptions object with the current list of perceptions
   */
  Perceptions get_perceptions() const;

protected:
  /**
   * @brief Executes a single cycle. Must be implemented by derived classes
   *
   * This method is periodically called by a timer to run the sensors logic
   */
  virtual void sensors_cycle() = 0;

  /**
   * @brief The list of perceptions from the Sensors module
   */
  Perceptions perceptions_ {};

private:
  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic sensors tasks cycle.
   */
  rclcpp::TimerBase::SharedPtr sensors_main_timer_;
};


/**
 * @class SimpleSensors
 * @brief This is only an example class that implements the interface defined in SensorsNodeBase
 *
 */
class SimpleSensors : public SensorsNodeBase
{
protected:
  /**
  * @brief Executes a single cycle. Must be implemented by derived classes.
  *
  * This method should compute the actual perceptions
  */
  virtual void sensors_cycle() override
  {
    // Example implementation for the control cycle
    perceptions_.clear();
    // perceptions_.push_back(...)
  }
};

}  // namespace easynav_sensors

#endif  // EASYNAV_SENSORS__SENSORS_NODE_BASE_HPP_
