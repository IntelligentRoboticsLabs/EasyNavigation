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
/// \brief Definition of the Perception struct and the Perceptions container used for sensor data input.


#include <string>
#include <vector>

#include "pcl/point_types.h"
#include "pcl/common/transforms.h"
#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"
#include "easynav_sensors/SensorsUtils.hpp"

namespace easynav
{


void
convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc)
{
  const size_t num_points = scan.ranges.size();

  if (pc.points.size() != num_points) {
    pc.points.resize(num_points);
  }

  pc.header.frame_id = scan.header.frame_id;
  pc.width = static_cast<uint32_t>(pc.points.size());
  pc.height = 1;
  pc.is_dense = false;

  for (size_t i = 0; i < num_points; ++i) {
    float range = scan.ranges[i];

    pcl::PointXYZ & point = pc.points[i];

    if (!std::isfinite(range) || range < scan.range_min || range > scan.range_max) {
      point.x = std::numeric_limits<float>::quiet_NaN();
      point.y = std::numeric_limits<float>::quiet_NaN();
      point.z = std::numeric_limits<float>::quiet_NaN();
    } else {
      float angle = scan.angle_min + i * scan.angle_increment;
      point.x = range * std::cos(angle);
      point.y = range * std::sin(angle);
      point.z = 0.0f;
    }
  }
}

void fuse_perceptions(
  const std::vector<std::shared_ptr<Perception>> & perceptions,
  const std::string & target_frame,
  tf2_ros::Buffer & tf_buffer,
  sensor_msgs::msg::PointCloud2 & output_msg)
{
  pcl::PointCloud<pcl::PointXYZ> merged;
  size_t total_points = 0;

  for (const auto & p : perceptions) {
    if (p && p->valid) {
      total_points += p->data.size();
    }
  }

  merged.points.reserve(total_points);
  merged.height = 1;
  merged.is_dense = false;
  merged.header.frame_id = target_frame;

  for (const auto & p : perceptions) {
    if (!p || !p->valid || p->data.empty()) {
      continue;
    }

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer.lookupTransform(
        target_frame, p->frame_id, tf2_ros::fromMsg(p->stamp), tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(
        rclcpp::get_logger("PerceptionFusion"),
        "TF transform from '%s' to '%s' failed: %s",
        p->frame_id.c_str(), target_frame.c_str(), ex.what());
      continue;
    }

    Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);

    pcl::PointCloud<pcl::PointXYZ> transformed;
    pcl::transformPointCloud(p->data, transformed, tf_eigen);

    merged.points.insert(merged.points.end(), transformed.begin(), transformed.end());
  }

  merged.width = merged.points.size();

  pcl::toROSMsg(merged, output_msg);
  output_msg.header.frame_id = target_frame;

  rclcpp::Time latest_stamp(0, 0, RCL_ROS_TIME);

  for (const auto & p : perceptions) {
    if (p && p->valid && p->stamp > latest_stamp) {
      latest_stamp = p->stamp;
    }
  }

  output_msg.header.stamp = latest_stamp;
}

template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::LaserScan>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception,
  rclcpp::CallbackGroup::SharedPtr cbg)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = cbg;

  return create_subscription<sensor_msgs::msg::LaserScan>(
    node,
    topic,
    rclcpp::SensorDataQoS().reliable(),
    [perception](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      convert(*msg, perception->data);

      perception->frame_id = msg->header.frame_id;
      perception->stamp = msg->header.stamp;
      perception->valid = true;
    }, options);
}

template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::PointCloud2>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception,
  rclcpp::CallbackGroup::SharedPtr cbg)
{
  rclcpp::SubscriptionOptions options;
  options.callback_group = cbg;

  return create_subscription<sensor_msgs::msg::PointCloud2>(
    node,
    topic,
    rclcpp::SensorDataQoS().reliable(),
    [perception](sensor_msgs::msg::PointCloud2::UniquePtr msg) {
      pcl::fromROSMsg(*msg, perception->data);

      perception->frame_id = msg->header.frame_id;
      perception->stamp = msg->header.stamp;
      perception->valid = true;
    }, options);
}


}  // namespace easynav
