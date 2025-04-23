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
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/Perceptions.hpp"

namespace easynav_sensors
{

template<typename MsgT>
rclcpp::SubscriptionBase::SharedPtr create_typed_subscription(
    rclcpp_lifecycle::LifecycleNode & node,
    const std::string & topic, 
    Perception & perception)
{
  std::cerr << "Generic transform" << std::endl;
  return nullptr;
}

template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::LaserScan>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic, 
  Perception & perception)
{
  return create_subscription<sensor_msgs::msg::LaserScan>(
    node,
    topic,
    rclcpp::SensorDataQoS().reliable(),
    [&perception](sensor_msgs::msg::LaserScan::UniquePtr msg) {
      const size_t num_points = msg->ranges.size();

      if (perception.data.points.size() != num_points) {
        perception.data.points.resize(num_points);
      }

      perception.data.header.frame_id = msg->header.frame_id;
      perception.data.width = static_cast<uint32_t>(perception.data.points.size());
      perception.data.height = 1;
      perception.data.is_dense = false;

      for (size_t i = 0; i < num_points; ++i) {
        float range = msg->ranges[i];

        pcl::PointXYZ & point = perception.data.points[i];

        if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
          point.x = std::numeric_limits<float>::quiet_NaN();
          point.y = std::numeric_limits<float>::quiet_NaN();
          point.z = std::numeric_limits<float>::quiet_NaN();
        } else {
          float angle = msg->angle_min + i * msg->angle_increment;
          point.x = range * std::cos(angle);
          point.y = range * std::sin(angle);
          point.z = 0.0f;
        }
      }
    });
}

template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::PointCloud2>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic, 
  Perception & perception)
{
  std::cerr << "PointCloud2 transform" << std::endl;
  return nullptr;
}


}  // namespace easynav_sensors
