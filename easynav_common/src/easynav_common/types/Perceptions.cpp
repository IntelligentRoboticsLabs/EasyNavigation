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
#include "easynav_common/RTTFBuffer.hpp"

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

sensor_msgs::msg::PointCloud2
perception_to_rosmsg(const Perception & perception)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(perception.data, msg);
  msg.header.frame_id = perception.frame_id;
  msg.header.stamp = perception.stamp;
  return msg;
}

sensor_msgs::msg::PointCloud2
points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & cloud)
{
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
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
      perception->new_data = true;
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
       perception->new_data = true;
    }, options);
}


PerceptionsOpsView::PerceptionsOpsView(const Perceptions & perceptions)
: perceptions_(perceptions), indices_(perceptions.size())
{
  for (std::size_t i = 0; i < perceptions.size(); ++i) {
    if (perceptions[i]) {
      indices_[i].indices.resize(perceptions[i]->data.size());
      std::iota(indices_[i].indices.begin(), indices_[i].indices.end(), 0);
    }
  }
}

PerceptionsOpsView::PerceptionsOpsView(Perceptions && perceptions)
: owned_(std::move(perceptions)), perceptions_(*owned_), indices_(perceptions_.size())
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (perceptions_[i]) {
      indices_[i].indices.resize(perceptions_[i]->data.size());
      std::iota(indices_[i].indices.begin(), indices_[i].indices.end(), 0);
    }
  }
}

PerceptionsOpsView &
PerceptionsOpsView::filter(
  const std::vector<double> & min_bounds,
  const std::vector<double> & max_bounds)
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i]) {continue;}

    const auto & cloud = perceptions_[i]->data;
    auto & indices = indices_[i].indices;

    std::size_t write_idx = 0;
    for (std::size_t read_idx = 0; read_idx < indices.size(); ++read_idx) {
      const auto & pt = cloud[indices[read_idx]];
      bool keep = true;
      if (!std::isnan(min_bounds[0]) && pt.x < min_bounds[0]) {keep = false;}
      if (!std::isnan(max_bounds[0]) && pt.x > max_bounds[0]) {keep = false;}
      if (!std::isnan(min_bounds[1]) && pt.y < min_bounds[1]) {keep = false;}
      if (!std::isnan(max_bounds[1]) && pt.y > max_bounds[1]) {keep = false;}
      if (!std::isnan(min_bounds[2]) && pt.z < min_bounds[2]) {keep = false;}
      if (!std::isnan(max_bounds[2]) && pt.z > max_bounds[2]) {keep = false;}

      if (keep) {
        indices[write_idx++] = indices[read_idx];
      }
    }

    indices.resize(write_idx);
  }

  return *this;
}

PerceptionsOpsView &
PerceptionsOpsView::downsample(double resolution)
{
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i]) {continue;}

    const auto & cloud = perceptions_[i]->data;
    auto & indices = indices_[i].indices;

    std::unordered_set<std::tuple<int, int, int>> voxel_set;
    std::size_t write_idx = 0;

    for (std::size_t read_idx = 0; read_idx < indices.size(); ++read_idx) {
      const auto & pt = cloud[indices[read_idx]];
      auto voxel = std::make_tuple(
        static_cast<int>(pt.x / resolution),
        static_cast<int>(pt.y / resolution),
        static_cast<int>(pt.z / resolution));

      if (voxel_set.insert(voxel).second) {
        indices[write_idx++] = indices[read_idx];
      }
    }

    indices.resize(write_idx);
  }

  return *this;
}

std::shared_ptr<PerceptionsOpsView>
PerceptionsOpsView::collapse(const std::vector<double> & collapse_dims) const
{
  Perceptions result;

  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i]) {continue;}

    auto collapsed = std::make_shared<Perception>();
    collapsed->valid = perceptions_[i]->valid;
    collapsed->frame_id = perceptions_[i]->frame_id;
    collapsed->stamp = perceptions_[i]->stamp;

    const auto & cloud = perceptions_[i]->data;
    for (int idx : indices_[i].indices) {
      auto pt = cloud[idx];
      if (!std::isnan(collapse_dims[0])) {pt.x = collapse_dims[0];}
      if (!std::isnan(collapse_dims[1])) {pt.y = collapse_dims[1];}
      if (!std::isnan(collapse_dims[2])) {pt.z = collapse_dims[2];}
      collapsed->data.push_back(pt);
    }

    result.push_back(collapsed);
  }

  return std::make_shared<PerceptionsOpsView>(std::move(result));
}

pcl::PointCloud<pcl::PointXYZ>
PerceptionsOpsView::as_points() const
{
  pcl::PointCloud<pcl::PointXYZ> output;
  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    if (!perceptions_[i]) {continue;}
    const auto & cloud = perceptions_[i]->data;
    for (int idx : indices_[i].indices) {
      output.push_back(cloud[idx]);
    }
  }
  return output;
}

const pcl::PointCloud<pcl::PointXYZ> &
PerceptionsOpsView::as_points(int idx) const
{
  return perceptions_[idx]->data;
}

std::shared_ptr<PerceptionsOpsView>
PerceptionsOpsView::fuse(
  const std::string & target_frame) const
{
  auto fused = std::make_shared<Perception>();
  fused->valid = true;
  fused->frame_id = target_frame;
  std::optional<rclcpp::Time> latest_stamp;

  for (std::size_t i = 0; i < perceptions_.size(); ++i) {
    const auto & p = perceptions_[i];
    if (!p || !p->valid || p->data.empty()) {continue;}

    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = RTTFBuffer::getInstance()->lookupTransform(
        target_frame, p->frame_id, tf2_ros::fromMsg(p->stamp), tf2::durationFromSec(0.1));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(rclcpp::get_logger("PerceptionsOpsView"), "TF failed: %s", ex.what());
      continue;
    }

    Eigen::Affine3d tf_eigen = tf2::transformToEigen(tf);
    pcl::PointCloud<pcl::PointXYZ> transformed;
    for (int idx : indices_[i].indices) {
      transformed.push_back(p->data[idx]);
    }

    pcl::transformPointCloud(transformed, transformed, tf_eigen);
    fused->data += transformed;

    if (!latest_stamp.has_value() || p->stamp > latest_stamp.value()) {
      latest_stamp = p->stamp;
    }
  }

  fused->stamp = latest_stamp.value_or(rclcpp::Time(0));

  Perceptions result;
  result.push_back(fused);

  return std::make_shared<PerceptionsOpsView>(std::move(result));
}

}  // namespace easynav
