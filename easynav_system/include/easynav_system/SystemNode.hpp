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
/// \brief Declaration of the SystemNode class, the central coordinator node for Easy Navigation components.

#ifndef EASYNAV_SYSTEM__SYSTEMNODE_HPP_
#define EASYNAV_SYSTEM__SYSTEMNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_system/GoalManager.hpp"

namespace easynav
{

/**
 * @struct SystemNodeInfo
 * @brief Structure holding runtime information for a subnode.
 */
struct SystemNodeInfo
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_ptr; ///< Shared pointer to the managed lifecycle node.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg;       ///< Associated real-time callback group.
};

/**
 * @class SystemNode
 * @brief ROS 2 lifecycle node coordinating all Easy Navigation components.
 *
 * Manages lifecycle transitions, real-time execution, and communication
 * between planner, controller, localizer, map manager, and sensor nodes.
 */
class SystemNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(SystemNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructor.
   * @param options Node options.
   */
  explicit SystemNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /// @brief Destructor.
  ~SystemNode();

  /**
   * @brief Configure the node.
   * @param state Lifecycle state.
   * @return SUCCESS if configuration succeeded.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if activation succeeded.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivate the node.
   * @param state Lifecycle state.
   * @return SUCCESS if deactivation succeeded.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleanup the node.
   * @param state Lifecycle state.
   * @return SUCCESS if cleanup succeeded.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shutdown the node.
   * @param state Lifecycle state.
   * @return SUCCESS if shutdown succeeded.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handle lifecycle transition error.
   * @param state Lifecycle state.
   * @return SUCCESS if error handled.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Get the real-time callback group.
   * @return Shared pointer to the callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Get all system nodes managed by this coordinator.
   * @return Map of node names to node information.
   */
  std::map<std::string, SystemNodeInfo> get_system_nodes();

  /**
   * @brief Real-time system cycle.
   */
  void system_cycle_rt();

  /**
   * @brief Non-real-time system cycle.
   */
  void system_cycle();

private:
  /// @brief Positional tolerance in meters.
  double position_tolerance_ {0.03};

  /// @brief Angular tolerance in radians.
  double angle_tolerance_ {0.01};

  /// @brief Real-time callback group.
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /// @brief Timer for non-real-time updates.
  rclcpp::TimerBase::SharedPtr system_main_nort_timer_;

  /// @brief Controller node.
  ControllerNode::SharedPtr controller_node_;

  /// @brief Localizer node.
  LocalizerNode::SharedPtr localizer_node_;

  /// @brief Maps manager node.
  MapsManagerNode::SharedPtr maps_manager_node_;

  /// @brief Planner node.
  PlannerNode::SharedPtr planner_node_;

  /// @brief Sensors node.
  SensorsNode::SharedPtr sensors_node_;

  /// @brief Shared navigation state.
  std::shared_ptr<NavState> nav_state_;

  /// @brief Goal manager.
  GoalManager::SharedPtr goal_manager_;

  /// @brief Publisher for velocity command (stamped).
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_stamped_;

  /// @brief Publisher for velocity command (legacy Twist).
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

}  // namespace easynav

#endif  // EASYNAV_SYSTEM__SYSTEMNODE_HPP_
