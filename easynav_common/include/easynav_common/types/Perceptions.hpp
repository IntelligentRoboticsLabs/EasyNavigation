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
/// \brief Defines data structures and utilities for representing and processing sensor perceptions.
/// Includes conversion between ROS messages and PCL point clouds, fusion of data, and tools for creating filtered views.

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
  bool new_data = false;                    ///< Whether new data has been received.
};

/**
 * @typedef Perceptions
 * @brief Alias for a vector of shared pointers to Perception objects.
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
 * @return The resulting PointCloud2 message.
 */
sensor_msgs::msg::PointCloud2 perception_to_rosmsg(const Perception & perception);

/**
 * @brief Converts a point cloud into a ROS PointCloud2 message.
 *
 * @param points The input points as a pcl::PointCloud<pcl::PointXYZ>.
 * @return The resulting PointCloud2 message.
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
 *
 * @param node Lifecycle node that owns the subscription.
 * @param topic Topic name to subscribe to.
 * @param perception Perception object where data will be stored.
 * @param cbg Callback group for the subscription.
 * @return Shared pointer to the created SubscriptionBase.
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
 *
 * @param node Lifecycle node that owns the subscription.
 * @param topic Topic name to subscribe to.
 * @param perception Perception object where data will be stored.
 * @param cbg Callback group for the subscription.
 * @return Shared pointer to the created SubscriptionBase.
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
   * @brief Constructs a view interface for the given Perceptions container.
   *
   * @param perceptions A constant reference to a vector of shared pointers to Perception instances.
   */
  explicit PerceptionsOpsView(const Perceptions & perceptions);

  /**
   * @brief Constructs a view by taking ownership of the Perceptions container.
   *
   * @param perceptions A rvalue reference to a vector of shared pointers to Perception instances.
   */
  explicit PerceptionsOpsView(Perceptions && perceptions);

  /**
   * @brief Filters all point clouds according to given bounds (x, y, z).
   *
   * NaN values in the bounds vectors disable filtering for the corresponding axis.
   *
   * @param min_bounds A vector of 3 doubles specifying minimum allowed values per axis. Use NAN to ignore.
   * @param max_bounds A vector of 3 doubles specifying maximum allowed values per axis. Use NAN to ignore.
   * @return Reference to self for method chaining.
   */
  PerceptionsOpsView & filter(
    const std::vector<double> & min_bounds,
    const std::vector<double> & max_bounds);

  /**
   * @brief Applies voxel grid downsampling to each perception using a specified resolution.
   *
   * @param resolution The voxel size in meters.
   * @return Reference to self for method chaining.
   */
  PerceptionsOpsView & downsample(double resolution);

  /**
   * @brief Collapses selected dimensions to fixed values.
   *
   * Any axis set to NaN will remain unchanged.
   *
   * @param collapse_dims A vector of 3 values representing the fixed coordinates per axis.
   * @return A shared pointer to a new PerceptionsOpsView created with the collapsed perception.
   */
  std::shared_ptr<PerceptionsOpsView> collapse(const std::vector<double> & collapse_dims) const;

  /**
   * @brief Returns the resulting points from the current view as a single flat point cloud.
   *
   * @return A pcl::PointCloud<pcl::PointXYZ> containing all selected points.
   */
  pcl::PointCloud<pcl::PointXYZ> as_points() const;

  /**
   * @brief Returns the resulting points for a specific perception index.
   *
   * @param idx The index of the perception.
   * @return A const reference to the filtered point cloud for the specified index.
   */
  const pcl::PointCloud<pcl::PointXYZ> & as_points(int idx) const;

  /**
   * @brief Fuses all selected perceptions into a single unified view, transforming them to a common frame.
   *
   * @param target_frame The frame into which all data should be transformed.
   * @return A shared pointer to a new PerceptionsOpsView with the fused result.
   */
  std::shared_ptr<PerceptionsOpsView> fuse(
    const std::string & target_frame) const;

  /**
   * @brief Returns the internal reference to the container of perceptions.
   *
   * @return Constant reference to the original Perceptions.
   */
  const Perceptions & get_perceptions() const {return perceptions_;}

private:
  std::optional<Perceptions> owned_;                     ///< Optional ownership of Perceptions if moved in.
  const Perceptions & perceptions_;                      ///< Reference to the original Perceptions container.
  std::vector<pcl::PointIndices> indices_;               ///< Selected point indices for each Perception.
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
