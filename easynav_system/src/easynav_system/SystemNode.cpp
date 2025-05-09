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
/// \brief Implementation of the SystemNode class.

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/SystemNode.hpp"

#include "easynav_controller/ControllerNode.hpp"
#include "easynav_localizer/LocalizerNode.hpp"
#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_planner/PlannerNode.hpp"
#include "easynav_sensors/SensorsNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"


namespace easynav
{

using namespace std::chrono_literals;

SystemNode::SystemNode(const rclcpp::NodeOptions & options)
: LifecycleNode("system_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  nav_state_ = std::make_shared<NavState>();

  controller_node_ = ControllerNode::make_shared(nav_state_);
  localizer_node_ = LocalizerNode::make_shared(nav_state_);
  maps_manager_node_ = MapsManagerNode::make_shared(nav_state_);
  planner_node_ = PlannerNode::make_shared(nav_state_);
  sensors_node_ = SensorsNode::make_shared();
}

SystemNode::~SystemNode()
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
SystemNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Configuring [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to configure [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  if (const auto res = goal_manager_.initialize(shared_from_this()); !res) {
    RCLCPP_ERROR(
      rclcpp::get_logger("SystemNode"),
      "Failed to initialize GoalManager: %s",
      res.error().c_str()
    );
    return CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Activating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to activate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  system_main_nort_timer_ = create_timer(300ms, std::bind(&SystemNode::system_cycle, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  for (auto & system_node : get_system_nodes()) {
    RCLCPP_INFO(get_logger(), "Deactivating [%s]", system_node.first.c_str());
    system_node.second.node_ptr->trigger_transition(
      lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);

    if (system_node.second.node_ptr->get_current_state().id() !=
      lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(get_logger(), "Unable to deactivate [%s]", system_node.first.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  system_main_nort_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SystemNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SystemNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
SystemNode::system_cycle_rt()
{
  auto start = now();
  bool trigger_perceptions = sensors_node_->cycle_rt();
  nav_state_->perceptions = sensors_node_->get_perceptions();

  bool trigger_localization = localizer_node_->cycle_rt(trigger_perceptions);
  nav_state_->odom = localizer_node_->get_odom();

  bool trigger_controller = controller_node_->cycle_rt(
    trigger_perceptions || trigger_localization);

  if (trigger_controller) {
    nav_state_->cmd_vel = controller_node_->get_cmd_vel();
    // vel_pub_->publish(nav_state_->cmd_vel);
  }

  std::cerr << "rt: " << (now() - start).seconds() << std::endl;
}

void
SystemNode::system_cycle()
{
  auto start = now();
  sensors_node_->cycle();


  nav_state_->perceptions = sensors_node_->get_perceptions();

  localizer_node_->cycle();

  nav_state_->odom = localizer_node_->get_odom();

  maps_manager_node_->cycle();
  nav_state_->maps = maps_manager_node_->get_maps();
  // nav_state_->dynamic_map = maps_manager_node_->get_dynamic_map();
  // nav_state_->goal = goal_;

  planner_node_->cycle();

  nav_state_->path = planner_node_->get_path();

  std::cerr << "nort: " << (now() - start).seconds() << std::endl;
}

std::map<std::string, SystemNodeInfo>
SystemNode::get_system_nodes()
{
  std::map<std::string, SystemNodeInfo> ret;

  ret[controller_node_->get_name()] = {controller_node_, controller_node_->get_real_time_cbg()};
  ret[localizer_node_->get_name()] = {localizer_node_, localizer_node_->get_real_time_cbg()};
  ret[maps_manager_node_->get_name()] = {maps_manager_node_, nullptr};
  ret[planner_node_->get_name()] = {planner_node_, nullptr};
  ret[sensors_node_->get_name()] = {sensors_node_, sensors_node_->get_real_time_cbg()};

  return ret;
}

}  // namespace easynav
