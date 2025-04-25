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
/// \brief Declaration of the MapsTypeBase template class, an abstract interface to represent map types.

#ifndef EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_
#define EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/**
 * @class MapsTypeBase
 * @brief Abstract base class for managing map data in Easy Navigation.
 *
 * This class defines an interface for modules responsible for maintaining
 * static and dynamic maps, including publishing and updating based on incoming perceptions.
 *
 */
class MapsTypeBase
{
public:
  /**
   * @brief Construct a new MapsTypeBase
   */
  MapsTypeBase() = default;

  /**
   * @brief Virtual destructor for MapsTypeBase
   */
  virtual ~MapsTypeBase() = default;

  /**
   * @brief Initialize the MapsTypeBase with a parent lifecycle node.
   *
   * This method should be called once the lifecycle node is configured.
   *
   * @param parent_node shared pointer to Lifecycle node that provides ROS interfaces.
   */
  void initialize(std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node);

  /**
   * @brief Publish the static map.
   *
   * This method is intended to be called when the static map should be made available,
   * e.g. during initialization or reconfiguration.
   */
  virtual void publish_static_map() = 0;

  /**
   * @brief Publish the current dynamic map.
   *
   * Called regularly or when dynamic elements in the environment are updated.
   */
  virtual void publish_dynamic_map() = 0;

  /**
   * @brief Update the internal dynamic map with new perception data.
   * @param perceptions A container of recent perception entries.
   */
  virtual void update_dynamic_map(const easynav::Perceptions & perceptions) = 0;

private:
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_; ///< Owning node that provides lifecycle context.
};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_
