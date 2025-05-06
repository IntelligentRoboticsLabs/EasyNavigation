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
#include "pcl/PointIndices.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace std
{

template<>
struct hash<std::tuple<int, int, int>>
{
  std::size_t operator()(const std::tuple<int, int, int> & key) const
  {
    std::size_t h1 = std::hash<int>()(std::get<0>(key));
    std::size_t h2 = std::hash<int>()(std::get<1>(key));
    std::size_t h3 = std::hash<int>()(std::get<2>(key));
    return h1 ^ (h2 << 1) ^ (h3 << 2);  // combinación simple
  }
};

}  // namespace std

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
  bool new_data = false;
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
 * @brief Converts points into a ROS PointCloud2 message.
 *
 * @param points The input points as a pcl::PointCloud<pcl::PointXYZ>
 * @return sensor_msgs::msg::PointCloud2 The resulting PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2 points_to_rosmsg(const pcl::PointCloud<pcl::PointXYZ> & points);

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
 * @class PerceptionsOpsView
 * @brief Provides efficient, non-destructive, chainable operations over a set of sensor perceptions.
 *
 * This class operates over a constant reference to a Perceptions container and uses point indices
 * to define views over the original point clouds, enabling efficient processing without modifying or copying data.
 * It supports filtering and downsampling. Collapsing and fusing produce new views.
 */
class PerceptionsOpsView
{
public:
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
    std::size_t operator()(const VoxelKey & key) const
    {
      std::size_t h1 = std::hash<int>{}(key.x);
      std::size_t h2 = std::hash<int>{}(key.y);
      std::size_t h3 = std::hash<int>{}(key.z);
      return h1 ^ (h2 << 1) ^ (h3 << 2);
    }
  };

  /**
   * @brief Constructs a view interface for the given Perceptions.
   * @param perceptions A constant reference to a vector of shared pointers to Perception instances.
   */
  explicit PerceptionsOpsView(const Perceptions & perceptions);
  explicit PerceptionsOpsView(Perceptions && perceptions);

  /**
   * @brief Filters all point clouds according to given bounds (x, y, z). NaN disables dimension filtering.
   *
   * @param min_bounds A vector of 3 doubles specifying minimum allowed values per axis. Use NAN to ignore.
   * @param max_bounds A vector of 3 doubles specifying maximum allowed values per axis. Use NAN to ignore.
   * @return Reference to self for chaining.
   */
  PerceptionsOpsView & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds);

  /**
   * @brief Applies voxel grid downsampling to each perception using a given resolution.
   *
   * @param resolution The voxel size in meters.
   * @return Reference to self for chaining.
   */
  PerceptionsOpsView & downsample(double resolution);

  /**
   * @brief Collapses selected dimensions to fixed values.
   *
   * Any dimension set to NaN will be left unchanged.
   *
   * @param collapse_dims A vector of 3 values representing the fixed coordinates per axis.
   * @return A shared pointer to a new PerceptionsOpsView created with the collapsed perception.
   */
  std::shared_ptr<PerceptionsOpsView> collapse(const std::vector<double> & collapse_dims) const;

  /**
   * @brief Returns the resulting points from the current view as a single flat vector.
   *        The returned vector is built with copies of the selected points.
   * @return A std::vector containing all filtered, downsampled, or collapsed points.
   */
  pcl::PointCloud<pcl::PointXYZ> as_points() const;

  /**
   * @brief Returns the resulting points from the current view as a reference to pcl::PointCloud<pcl::PointXYZ>.
   *        The returned vector is built with copies of the selected points.
   * @param idx The index to the perception to return data.
   * @return A pcl::PointCloud<pcl::PointXYZ> all filtered, downsampled, or collapsed points.
   */
  const pcl::PointCloud<pcl::PointXYZ> & as_points(int idx) const;

  /**
   * @brief Fuses the current view into a single Perception, transforming each cloud into the target frame.
   *
   * The resulting Perception includes all valid points from the selected indices, transformed via tf.
   *
   * @param target_frame The desired target frame.
   * @param tf_buffer TF buffer used for coordinate transformations.
   * @return A shared pointer to a new PerceptionsOpsView wrapping the fused perception.
   */
  std::shared_ptr<PerceptionsOpsView> fuse(
    const std::string & target_frame,
    tf2_ros::Buffer & tf_buffer) const;

private:
  std::optional<Perceptions> owned_;
  const Perceptions & perceptions_;  ///< Reference to the original Perceptions container.
  std::vector<pcl::PointIndices> indices_;  ///< Selected point indices for each Perception.
};


}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
