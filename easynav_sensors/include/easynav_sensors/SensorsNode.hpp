// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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

#include "easynav_sensors/Perceptions.hpp"

namespace easynav_sensors
{

/**
 * @class SensorsNode
 * @brief ROS 2 lifecycle node that manages sensor fusion for the Easy Navigation system.
 *
 * This node handles the collection, transformation, and fusion of perception data from multiple sensor sources,
 * managing lifecycle transitions and real-time scheduling of tasks.
 */
class SensorsNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SensorsNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a SensorsNode lifecycle node with the specified options.
   * @param options Node options to configure the SensorsNode node.
   */
  explicit SensorsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the SensorsNode object.
   */
  ~SensorsNode();

  /**
   * @brief Configures the SensorsNode node.
   * Declares parameters, initializes tf buffers, and prepares internal structures.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the SensorsNode node.
   * Starts periodic tasks such as perception fusion and publishing.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the SensorsNode node.
   * Stops periodic tasks and publishers.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the SensorsNode node.
   * Releases resources such as subscriptions, timers, and tf listeners.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the SensorsNode node.
   * Performs final resource release and shutdown procedures.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the SensorsNode node.
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
   * @brief Retrieves the current set of active perceptions.
   *
   * @return Const reference to the list of perceptions.
   */
  const Perceptions & get_perceptions() const {return perceptions_;}

private:
  /**
   * @brief Callback group intended for real-time sensor processing tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic sensor fusion cycle.
   */
  rclcpp::TimerBase::SharedPtr sensors_main_timer_;

  /**
   * @brief Buffer storing transformations between frames.
   */
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  /**
   * @brief Transform listener that populates the tf buffer.
   */
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  /**
   * @brief Publisher for fused perception messages.
   */
  rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::PointCloud2>::SharedPtr percept_pub_;

  /**
   * @brief The latest fused perception message to be published.
   */
  sensor_msgs::msg::PointCloud2 perecption_msg_;

  /**
   * @brief Executes one cycle of real-time system operations.
   *
   * This function is called periodically by the real-time timer to manage control,
   * localization, planning, and other tightly coupled tasks.
   */
  void sensors_cycle_rt();

  /**
   * @brief Executes one cycle of non-real-time system operations.
   *
   * This function manages background tasks not requiring strict real-time execution.
   */
  void sensors_cycle_nort();

  /**
   * @brief Container storing current active perceptions.
   */
  Perceptions perceptions_;

  /**
   * @brief Maximum age (in seconds) after which a perception is discarded.
   */
  double forget_time_;

  /**
   * @brief Target frame to which all perceptions will be transformed before fusion.
   */
  std::string perception_default_frame_;
};

}  // namespace easynav_sensors

#endif  // EASYNAV_SENSORS__SENSORNODE_HPP_
