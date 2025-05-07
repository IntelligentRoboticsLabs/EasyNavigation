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

namespace easynav
{

using namespace std::chrono_literals;

PlannerNode::PlannerNode(
  const std::shared_ptr<const NavState> & nav_state,
  const rclcpp::NodeOptions & options)
: LifecycleNode("planner_node", options),
  nav_state_(nav_state)
{
  planner_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::PlannerMethodBase>>(
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
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
PlannerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  try {
    planner_method_ = planner_loader_->createSharedInstance("easynav_planner/DummyPlanner");
    auto result = planner_method_->initialize(shared_from_this(), "dummy_planner");
    if (!result) {
      RCLCPP_ERROR(get_logger(),
        "Unable to initialize [dummy_planner]. Error: %s", result.error().c_str());
      return CallbackReturnT::FAILURE;
    }
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Unable to load plugin easynav::DummyPlanner. Error: %s", ex.what());
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
  return planner_method_->get_path();
}

void
PlannerNode::cycle()
{
  planner_method_->internal_update(*nav_state_);
}

}  // namespace easynav
