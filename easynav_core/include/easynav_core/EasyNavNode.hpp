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

#ifndef EASYNAV_CORE__EASYNAVNODE_HPP_
#define EASYNAV_CORE__EASYNAVNODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class EasyNavNode
 * @brief Represents the main node that manages navigation.
 */
class EasyNavNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(EasyNavNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a EasyNavNode lifecycle node with the specified options.
   * @param options Node options to configure the EasyNavNode node.
   */
  explicit EasyNavNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Configures the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the EasyNavNode node.
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * This callback group can be used to assign callbacks that require
   * low latency or have real-time constraints.
   *
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg() {return realtime_cbg_;}

private:
  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__EASYNAVNODE_HPP_
