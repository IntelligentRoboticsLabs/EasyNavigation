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
/// \brief uTILITY FUNCTIONS for handling sensor perceptions, including point cloud conversion, fusion, and subscription creation.

#ifndef EASYNAV_SENSORS__SENSORSUTILS_HPP_
#define EASYNAV_SENSORS__SENSORSUTILS_HPP_

#include <string>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "tf2_ros/buffer.h"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/**
 * @brief Converts a LaserScan message into a PCL point cloud.
 *
 * @param scan The input LaserScan message.
 * @param pc The output PCL point cloud to be filled.
 */
void convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc);

/**
 * @brief Fuses a collection of Perceptions into a single PointCloud2, transforming all frames into a common target frame.
 *
 * Each perception is checked for validity. Valid perceptions are transformed using tf2
 * into the specified target frame before being merged.
 *
 * @param perceptions Vector of shared pointers to Perception objects.
 * @param target_frame Target frame ID into which all clouds will be transformed.
 * @param tf_buffer A tf2 buffer used to lookup and perform frame transformations.
 * @param output_msg Output fused PointCloud2 message.
 */
void fuse_perceptions(
  const Perceptions & perceptions,
  const std::string & target_frame,
  tf2_ros::Buffer & tf_buffer,
  sensor_msgs::msg::PointCloud2 & output_msg);

/**
 * @brief Creates a subscription for a given sensor message type and binds it to a Perception object.
 *
 * @tparam MsgT The sensor message type (e.g., LaserScan, PointCloud2).
 * @param node Reference to the LifecycleNode where the subscription is created.
 * @param topic Name of the ROS 2 topic to subscribe to.
 * @param perception Shared pointer to the Perception structure that will store received data.
 * @param cbg Callback group to assign the subscription to.
 * @return Shared pointer to the created SubscriptionBase.
 */
template<typename MsgT>
rclcpp::SubscriptionBase::SharedPtr create_typed_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception,
  rclcpp::CallbackGroup::SharedPtr cbg
);

}  // namespace easynav

#endif  // EASYNAV_SENSORS__SENSORSUTILS_HPP_
