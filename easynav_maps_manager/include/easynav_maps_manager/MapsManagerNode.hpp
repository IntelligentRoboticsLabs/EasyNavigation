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
/// \brief Declaration of the MapsManagerNode class, a ROS 2 lifecycle node for map handling in Easy Navigation.

#ifndef EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
#define EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_common/types/MapTypeBase.hpp"
#include "easynav_core/MapsManagerBase.hpp"
#include "pluginlib/class_loader.hpp"

namespace easynav
{

/**
 * @class MapsManagerNode
 * @brief ROS 2 lifecycle node that manages map-related plugins in Easy Navigation.
 *
 * Handles lifecycle transitions, plugin loading, and periodic map updates.
 */
class MapsManagerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(MapsManagerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param nav_state Shared pointer to navigation state.
   * @param options Node options.
   */
  explicit MapsManagerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~MapsManagerNode();

  /**
   * @brief Configure the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if configuration succeeds.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if activation succeeds.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if deactivation succeeds.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if cleanup succeeds.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Current lifecycle state.
   * @return SUCCESS if shutdown succeeds.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle errors during lifecycle transitions.
   * @param state Current lifecycle state.
   * @return SUCCESS if error was handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Execute one update cycle (non real-time).
   */
  void cycle();

  /**
   * @brief Get the currently available maps.
   * @return Map of map IDs to map instances.
   */
  std::map<std::string, std::shared_ptr<MapsTypeBase>> get_maps() {return maps_;}

private:
  /// @brief Active map instances.
  std::map<std::string, std::shared_ptr<MapsTypeBase>> maps_;

  /// @brief Plugin loader for map manager implementations.
  std::unique_ptr<pluginlib::ClassLoader<MapsManagerBase>> maps_manager_loader_;

  /// @brief Active map manager plugins.
  std::vector<std::shared_ptr<MapsManagerBase>> maps_managers_;

  /// @brief Shared navigation state reference.
  const std::shared_ptr<const NavState> nav_state_;
};

}  // namespace easynav

#endif  // EASYNAV_MAPSMANAGER__MAPSMANAGERNODE_HPP_
