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
/// \brief Declaration of the LocalizerNode class, a ROS 2 lifecycle node for localization tasks in Easy Navigation.

#ifndef EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
#define EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_

#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "pluginlib/class_loader.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"
#include "easynav_common/types/NavState.hpp"

namespace easynav
{

/**
 * @class LocalizerNode
 * @brief ROS 2 lifecycle node that manages localization for the Easy Navigation system.
 *
 * This node serves as a runtime interface between the ROS 2 ecosystem and the
 * localization method plugin in EasyNav. It manages the lifecycle transitions,
 * real-time control cycles, and dynamic loading of the localization method.
 */
class LocalizerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  RCLCPP_SMART_PTR_DEFINITIONS(LocalizerNode)
  using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Constructs a LocalizerNode lifecycle node with the specified options.
   * @param options Node options to configure the LocalizerNode node.
   */
  explicit LocalizerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroys the LocalizerNode object.
   */
  ~LocalizerNode();

  /**
   * @brief Configures the LocalizerNode node.
   *
   * Declares and loads parameters, initializes the plugin loader, and instantiates
   * the selected localization method.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if configuration is successful.
   */
  CallbackReturnT on_configure(const rclcpp_lifecycle::State & state);

  /**
   * @brief Activates the LocalizerNode node.
   *
   * Starts timers and prepares the node to begin executing control cycles.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if activation is successful.
   */
  CallbackReturnT on_activate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Deactivates the LocalizerNode node.
   *
   * Stops timers and disables external interfaces without releasing resources.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS if deactivation is successful.
   */
  CallbackReturnT on_deactivate(const rclcpp_lifecycle::State & state);

  /**
   * @brief Cleans up the LocalizerNode node.
   *
   * Releases allocated resources and resets internal state.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating cleanup is complete.
   */
  CallbackReturnT on_cleanup(const rclcpp_lifecycle::State & state);

  /**
   * @brief Shuts down the LocalizerNode node.
   *
   * Final stage of the lifecycle. Similar to cleanup, but called during shutdown.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating shutdown is complete.
   */
  CallbackReturnT on_shutdown(const rclcpp_lifecycle::State & state);

  /**
   * @brief Handles errors in the LocalizerNode node.
   *
   * Called when a lifecycle transition fails, allowing for fallback or recovery behavior.
   *
   * @param state The current lifecycle state.
   * @return CallbackReturnT::SUCCESS indicating error handling is complete.
   */
  CallbackReturnT on_error(const rclcpp_lifecycle::State & state);

  /**
   * @brief Returns the real-time callback group.
   *
   * This callback group is reserved for timers or subscriptions with strict timing constraints.
   *
   * @return Shared pointer to the real-time callback group.
   */
  rclcpp::CallbackGroup::SharedPtr get_real_time_cbg();

  /**
   * @brief Get the current localization state.
   *
   * Retrieves the most recent odometry estimate produced by the localization method.
   *
   * @return An Odometry message representing the current robot pose and velocity.
   */
  [[nodiscard]] nav_msgs::msg::Odometry get_odom() const;

  /**
   * @brief Executes one cycle of real-time localization tasks.
   *
   * Typically called from a real-time timer. Invokes the plugin's update method.
   */
  bool localizer_cycle_rt(bool trigger = false);

private:
  /**
   * @brief Executes one cycle of non-real-time localization tasks.
   *
   * May be used for diagnostics, debugging, or deferred operations.
   */
  void localizer_cycle_nort();

  /**
   * @brief Callback group intended for real-time tasks.
   */
  rclcpp::CallbackGroup::SharedPtr realtime_cbg_;

  /**
   * @brief Timer that triggers the periodic localization cycle.
   */
  rclcpp::TimerBase::SharedPtr localizer_main_timer_;

  /**
   * @brief Pointer to the dynamically loaded localization method.
   */
  std::shared_ptr<LocalizerMethodBase> localizer_method_ {nullptr};

  /**
   * @brief Current navigation state passed to the localization plugin.
   */
  const std::shared_ptr<const NavState> nav_state_;

  /**
   * @brief Pluginlib loader for LocalizerMethodBase plugins.
   *
   * Used to dynamically instantiate the localization method based on configuration.
   */
  std::unique_ptr<pluginlib::ClassLoader<easynav::LocalizerMethodBase>> localizer_loader_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__LOCALIZERNODE_HPP_
