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
/// \brief Implementation of the PlannerNode class.

#include "nav_msgs/msg/path.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_planner/PlannerNode.hpp"
#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

using namespace std::chrono_literals;

PlannerNode::PlannerNode(
  const std::shared_ptr<const NavState> & nav_state,
  const rclcpp::NodeOptions & options)
: LifecycleNode("planner_node", options),
  nav_state_(nav_state)
{
  planner_loader_ = std::make_unique<pluginlib::ClassLoader<PlannerMethodBase>>(
    "easynav_core", "easynav::PlannerMethodBase");

}

PlannerNode::~PlannerNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  std::vector<std::string> planner_types;
  get_parameter("planner_types", planner_types);
  for (const auto & planner_type : planner_types) {
    planner_loader_->unloadLibraryForClass(planner_type);
  }
  planner_method_ = nullptr;
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PlannerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> planner_types;
  declare_parameter("planner_types", planner_types);
  get_parameter("planner_types", planner_types);

  if (planner_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one planner.  [%lu] found", planner_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & planner_type : planner_types) {
    std::string plugin;
    declare_parameter(planner_type + std::string(".plugin"), plugin);
    get_parameter(planner_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading PlannerMethodBase %s [%s]", planner_type.c_str(), plugin.c_str());

      planner_method_ = planner_loader_->createSharedInstance(plugin);

      auto result = planner_method_->initialize(shared_from_this(), planner_type);

      if (!result) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded PlannerMethodBase %s [%s]", planner_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin %s. Error: %s", plugin.c_str(), ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
PlannerNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

nav_msgs::msg::Path
PlannerNode::get_path() const
{
  if (planner_method_ == nullptr) {
    return nav_msgs::msg::Path();
  }

  return planner_method_->get_path();
}

void
PlannerNode::cycle()
{
  if (planner_method_ == nullptr) {return;}

  planner_method_->internal_update(*nav_state_);
}

}  // namespace easynav
