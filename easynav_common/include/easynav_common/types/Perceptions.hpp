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
/// \brief Definitions for a unified Perception type.

#ifndef EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
#define EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_

#include <memory>
#include <string>
#include <vector>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"

#include "rclcpp/time.hpp"

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
};

/**
 * @typedef Perceptions
 * @brief A container of multiple Perception entries, each possibly from different sources.
 */
using Perceptions = std::vector<std::shared_ptr<Perception>>;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__PERCEPTIONS_HPP_
