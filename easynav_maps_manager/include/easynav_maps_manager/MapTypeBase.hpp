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
/// \brief Declaration of the MapsTypeBase template class, an abstract interface for managing static and dynamic maps.

#ifndef EASYNAV_MAPSMANAGER__MAPSTYPEBASE_HPP_
#define EASYNAV_MAPSMANAGER__MAPSTYPEBASE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_sensors/Perceptions.hpp"

namespace easynav_maps_manager
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
  /**map_
   * @brief Construct a new MapsTypeBase.
   * @param parent_node Lifecycle node that owns this manager and provides ROS interfaces.
   */
  MapsTypeBase(rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node)
  : parent_node_(parent_node) {}

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
  virtual void update_dynamic_map(const easynav_sensors::Perceptions & perceptions) = 0;

  /**
   * @brief Get a read-only shared pointer to the internal map.
   * 
   * The returned pointer is `const` to prevent external modification of the map content.
   * 
   * @return std::shared_ptr<const T> Immutable pointer to the internal map.
   */
  // We have to see how to do this without templates
  // std::shared_ptr<const T> get_map() const { return map_; }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr parent_node_; ///< Owning node that provides lifecycle context.
  std::shared_ptr<T> map_; ///< Internal representation of the managed map.
};

}  // namespace easynav_maps_manager

#endif  // EASYNAV_MAPSMANAGER__MAPSTYPEBASE_HPP_
