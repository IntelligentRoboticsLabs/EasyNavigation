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
/// \brief Definitions for handling sensor perceptions, including point cloud conversion, fusion, and subscription creation.

#ifndef EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_

#include <string>
#include <vector>
#include <optional>

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @struct Perception
 * @brief Represents a single sensor perception used for mapping and localization.
 *
 * Contains point cloud data, associated timestamp, and frame information,
 * as well as metadata regarding its validity and the subscription that generated it.
 */
struct Perception
{
  pcl::PointCloud<pcl::PointXYZ> data;      ///< Point cloud data.
  rclcpp::Time stamp;                       ///< Timestamp of the perception.
  std::string frame_id;                     ///< Frame ID associated with the data.
  bool valid = false;                       ///< Whether the perception is valid or usable.
  rclcpp::SubscriptionBase::SharedPtr subscription; ///< ROS 2 subscription linked to the data source.
};

/**
 * @typedef Perceptions
 * @brief A container of multiple Perception entries, each possibly from different sources.
 */
using Perceptions = std::vector<std::shared_ptr<Perception>>;

/**
 * @brief Converts a LaserScan message into a PCL point cloud.
 *
 * @param scan The input LaserScan message.
 * @param pc The output PCL point cloud to be filled.
 */
void convert(const sensor_msgs::msg::LaserScan & scan, pcl::PointCloud<pcl::PointXYZ> & pc);

/**
 * @brief Converts a Perception into a ROS PointCloud2 message.
 *
 * @param perception The input Perception.
 * @return sensor_msgs::msg::PointCloud2 The resulting PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2 perception_to_rosmsg(const Perception & perception);

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
  rclcpp::CallbackGroup::SharedPtr cbg);

/**
 * @brief Specialization of create_typed_subscription for LaserScan messages.
 */
template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::LaserScan>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception,
  rclcpp::CallbackGroup::SharedPtr cbg);

/**
 * @brief Specialization of create_typed_subscription for PointCloud2 messages.
 */
template<>
rclcpp::SubscriptionBase::SharedPtr
create_typed_subscription<sensor_msgs::msg::PointCloud2>(
  rclcpp_lifecycle::LifecycleNode & node,
  const std::string & topic,
  std::shared_ptr<Perception> perception,
  rclcpp::CallbackGroup::SharedPtr cbg);

/**
 * @class PerceptionsOps
 * @brief Helper class providing efficient, chainable operations on Perceptions.
 *
 * Allows filtering, fusing, collapsing, and downsampling of point clouds
 * within the Perceptions container with performance in mind and fluent syntax.
 */
class PerceptionsOps
{
public:
  /**
   * @brief Constructs an operation interface over a mutable reference to a Perceptions container.
   * @param perceptions The Perceptions container to operate on.
   */
  explicit PerceptionsOps(Perceptions & perceptions);

  /**
   * @brief Constructs an operation interface from a const Perceptions container, taking ownership.
   *        The internal container is copied and kept alive inside the operator object.
   * @param perceptions The Perceptions container to copy and own.
   */
  explicit PerceptionsOps(const Perceptions & perceptions);

  /**
   * @brief Returns a deep copy of the current Perceptions container.
   * @return A cloned Perceptions object (shared_ptrs with copied PCL data).
   */
  Perceptions clone() const;

  /**
   * @brief Fuses all valid perceptions into a single point cloud, transforming to the given target frame.
   *
   * The fused result replaces all previous entries, and is stored in a new single Perception.
   *
   * @param target_frame Target coordinate frame for transformation.
   * @param tf_buffer The TF buffer used to resolve transformations.
   * @return Reference to this object (for method chaining).
   */
  PerceptionsOps & fuse(const std::string & target_frame, tf2_ros::Buffer & tf_buffer);

  /**
   * @brief Filters all point clouds based on spatial bounds.
   *
   * Only keeps points within [min_bounds, max_bounds]. Any dimension set to NaN is ignored.
   *
   * @param min_bounds Minimum (x, y, z) bounds. Use NaN to skip a dimension.
   * @param max_bounds Maximum (x, y, z) bounds. Use NaN to skip a dimension.
   * @return Reference to this object (for method chaining).
   */
  PerceptionsOps & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds);

  /**
   * @brief Collapses one or more dimensions of all point clouds to a fixed value.
   *
   * Any dimension specified with a numeric value is overwritten. NaN leaves the dimension unchanged.
   *
   * @param collapse_dims Vector of 3 values for (x, y, z). Use NaN to skip.
   * @return Reference to this object (for method chaining).
   */
  PerceptionsOps & collapse(const std::vector<double> & collapse_dims);

  /**
   * @brief Applies voxel grid filtering to all point clouds in the Perceptions container.
   *
   * @param resolution The voxel size to use for downsampling (in meters).
   * @return Reference to this object (for method chaining).
   */
  PerceptionsOps & downsample(double resolution);

  /**
   * @brief Returns a const reference to the internal Perceptions container.
   * @return Const reference to Perceptions.
   */
  const Perceptions & get() const;

  /**
   * @brief Returns a mutable reference to the internal Perceptions container.
   * @return Reference to Perceptions.
   */
  Perceptions & get();

  /**
   * @brief Returns the point cloud data from the (fused) single Perception.
   *
   * This is only valid after a `fuse()` operation that results in a single entry.
   *
   * @return Const reference to the fused pcl::PointCloud<pcl::PointXYZ>.
   */
  const pcl::PointCloud<pcl::PointXYZ> & fused_data() const;

  /**
   * @brief Returns the (fused) single Perception.
   *
   * This is only valid after a `fuse()` operation that results in a single entry.
   *
   * @return Const reference to a fused Perception.
   */
  const Perception & fused_perception() const;

private:
  std::optional<Perceptions> owned_;  ///< Owned container in case of copy construction.
  Perceptions & perceptions_;         ///< Reference to the underlying Perceptions container.
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
