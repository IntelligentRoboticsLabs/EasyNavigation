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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_core/EasyNavNode.hpp"

namespace easynav
{

/**
 * @brief Constructs a EasyNavNode lifecycle node with the specified options.
 * @param options Node options to configure the EasyNavNode node.
 */
EasyNavNode::EasyNavNode(const rclcpp::NodeOptions & options)
: LifecycleNode("easynav", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * @brief Configures the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS if configuration is successful.
 */
CallbackReturnT
EasyNavNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief Activates the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS if activation is successful.
 */
CallbackReturnT
EasyNavNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief Deactivates the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS if deactivation is successful.
 */
CallbackReturnT
EasyNavNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief Cleans up the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
 */
CallbackReturnT
EasyNavNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief Shuts down the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
 */
CallbackReturnT
EasyNavNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

/**
 * @brief Handles errors in the EasyNavNode node.
 * @param state The current lifecycle state.
 * @return CallbackReturnT::SUCCESS indicating error handling is complete.
 */
CallbackReturnT
EasyNavNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

}  // namespace easynav
