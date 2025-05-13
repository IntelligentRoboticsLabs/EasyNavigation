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
/// \brief Declaration of the SensorsNode class, a ROS 2 lifecycle node for sensor fusion tasks in Easy Navigation.

#ifndef EASYNAV_SENSORS__SENSORNODE_HPP_
#define EASYNAV_SENSORS__SENSORNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class SensorsNode
 * @brief ROS 2 lifecycle node that manages sensor fusion in Easy Navigation.
 *
 * Collects, transforms, and publishes fused perception data from multiple sources.
 */
class SensorsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SensorsNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Node configuration options.
   */
  explicit SensorsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~SensorsNode();

  /**
   * @brief Configure the node.
   * @param state Lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup the node.
   * @param state Lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition errors.
   * @param state Lifecycle state.
   * @return SUCCESS if error was handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the callback group for real-time tasks.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Get the current set of perceptions.
   * @return Copy of the internal Perceptions container.
   */
  const Perceptions get_perceptions() const {return perceptions_;}

  /**
   * @brief Run one real-time sensor processing cycle.
   * @param trigger Force execution regardless of frequency.
   * @return True if cycle executed.
   */
  bool cycle_rt(bool trigger = false);

  /**
   * @brief Run one non-real-time processing cycle.
   */
  void cycle();

private:
  /// @brief Callback group for real-time operations.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Publisher for the fused perception point cloud.
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr percept_pub_;

  /// @brief Last fused perception message.
  sensor_msgs::msg::PointCloud2 perecption_msg_;

  /// @brief Current set of active perceptions.
  Perceptions perceptions_;

  /// @brief Maximum time (seconds) a perception remains valid.
  double forget_time_;

  /// @brief Target frame for perception fusion.
  std::string perception_default_frame_;
};

}  // namespace easynav

#endif  // EASYNAV_SENSORS__SENSORNODE_HPP_
