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
#include <tuple>
#include <functional>  // Por si acaso

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_common/types/Perceptions.hpp"


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

/**
 * @brief Converts a Perception into a ROS PointCloud2 message.
 *
 * @param perception The input Perception.
 * @return sensor_msgs::msg::PointCloud2 The resulting PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2
perception_to_rosmsg(const Perception & perception)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(perception.data, msg);
  msg.header.frame_id = perception.frame_id;
  msg.header.stamp = perception.stamp;
  return msg;
}

template<typename MsgT>
rclcpp::SubscriptionBase::SharedPtr create_typed_subscription(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception)
{
  return nullptr;
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

PerceptionsOps::PerceptionsOps(Perceptions & perceptions)
: perceptions_(perceptions)
{}

PerceptionsOps::PerceptionsOps(const Perceptions & perceptions)
: owned_(perceptions), perceptions_(*owned_)
{}

Perceptions
PerceptionsOps::clone() const
{
  Perceptions copy;
  for (const auto & p : perceptions_) {
    auto new_p = std::make_shared<Perception>();
    new_p->stamp = p->stamp;
    new_p->frame_id = p->frame_id;
    new_p->valid = p->valid;
    new_p->data = p->data;
    copy.push_back(new_p);
  }
  return copy;
}

PerceptionsOps &
PerceptionsOps::fuse(
  const std::string & target_frame,
  tf2_ros::Buffer & tf_buffer)
{
  pcl::PointCloud<pcl::PointXYZ> merged;
  std::optional<rclcpp::Time> latest_stamp;

  for (const auto & p : perceptions_) {
    if (!p || !p->valid || p->data.empty()) {continue;}

    geometry_msgs::msg::TransformStamped tf;

    try {
      tf = tf_buffer.lookupTransform(
        target_frame, p->frame_id, tf2_ros::fromMsg(p->stamp), tf2::durationFromSec(0.0));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(rclcpp::get_logger("PerceptionsOps"), "TF failed: %s", ex.what());
      continue;
    }

    Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);
    pcl::PointCloud<pcl::PointXYZ> transformed;

    pcl::transformPointCloud(p->data, transformed, tf_eigen);

    merged += transformed;
    if (!latest_stamp.has_value() || p->stamp > latest_stamp.value()) {
      latest_stamp = p->stamp;
    }
  }


  auto fused = std::make_shared<Perception>();
  fused->data = merged;
  fused->stamp = latest_stamp.value_or(rclcpp::Time(0));
  fused->frame_id = target_frame;
  fused->valid = true;

  perceptions_.clear();
  perceptions_.push_back(fused);

  return *this;
}


PerceptionsOps &
PerceptionsOps::filter(
  const std::vector<double> & min_bounds,
  const std::vector<double> & max_bounds)
{
  for (auto & p : perceptions_) {
    if (!p || !p->valid) {continue;}

    pcl::PointCloud<pcl::PointXYZ> filtered;
    for (const auto & pt : p->data.points) {
      bool keep = true;
      if (!std::isnan(min_bounds[0]) && pt.x < min_bounds[0]) {keep = false;}
      if (!std::isnan(max_bounds[0]) && pt.x > max_bounds[0]) {keep = false;}
      if (!std::isnan(min_bounds[1]) && pt.y < min_bounds[1]) {keep = false;}
      if (!std::isnan(max_bounds[1]) && pt.y > max_bounds[1]) {keep = false;}
      if (!std::isnan(min_bounds[2]) && pt.z < min_bounds[2]) {keep = false;}
      if (!std::isnan(max_bounds[2]) && pt.z > max_bounds[2]) {keep = false;}
      if (keep) {filtered.points.push_back(pt);}
    }
    filtered.width = filtered.points.size();
    filtered.height = 1;
    p->data = std::move(filtered);
  }

  return *this;
}

PerceptionsOps &
PerceptionsOps::collapse(const std::vector<double> & fixed_coords)
{
  for (auto & p : perceptions_) {
    if (!p || !p->valid) {continue;}
    for (auto & pt : p->data.points) {
      if (!std::isnan(fixed_coords[0])) {pt.x = fixed_coords[0];}
      if (!std::isnan(fixed_coords[1])) {pt.y = fixed_coords[1];}
      if (!std::isnan(fixed_coords[2])) {pt.z = fixed_coords[2];}
    }
  }
  return *this;
}

struct VoxelKey
{
  int x, y, z;
  bool operator==(const VoxelKey & other) const
  {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash
{
  std::size_t operator()(const VoxelKey & k) const
  {
    return ((std::hash<int>()(k.x) ^ (std::hash<int>()(k.y) << 1)) >> 1) ^ (std::hash<int>()(k.z) <<
           1);
  }
};

PerceptionsOps &
PerceptionsOps::downsample(double resolution)
{
  for (auto & p : perceptions_) {
    if (!p || !p->valid) {continue;}

    std::unordered_set<VoxelKey, VoxelKeyHash> seen;
    pcl::PointCloud<pcl::PointXYZ> filtered;

    for (const auto & pt : p->data.points) {
      int xi = static_cast<int>(std::floor(pt.x / resolution));
      int yi = static_cast<int>(std::floor(pt.y / resolution));
      int zi = static_cast<int>(std::floor(pt.z / resolution));
      VoxelKey key{xi, yi, zi};
      if (seen.insert(key).second) {
        filtered.points.push_back(pt);
      }
    }

    filtered.width = filtered.points.size();
    filtered.height = 1;
    p->data = std::move(filtered);
  }

  return *this;
}

const pcl::PointCloud<pcl::PointXYZ> &
PerceptionsOps::fused_data() const
{
  if (perceptions_.size() != 1) {
    throw std::runtime_error("fused_data() requires exactly one perception.");
  }
  return perceptions_[0]->data;
}

const Perception &
PerceptionsOps::fused_perception() const
{
  if (perceptions_.size() != 1) {
    throw std::runtime_error("fused_data() requires exactly one perception.");
  }
  return *perceptions_[0];
}

}  // namespace easynav
