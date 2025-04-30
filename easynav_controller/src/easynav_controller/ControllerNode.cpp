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
/// \brief Implementation of the ControllerNode class.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_controller/ControllerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

ControllerNode::ControllerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  controller_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::ControllerMethodBase>>(
    "easynav_core", "easynav::ControllerMethodBase");
}


ControllerNode::~ControllerNode()
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
ControllerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  try {
    controller_method_ =
      controller_loader_->createSharedInstance("easynav_controller/DummyController");
    if (!controller_method_->initialize(shared_from_this(), "dummy_controller")) {
      RCLCPP_ERROR(get_logger(),
        "Unable to configure plugin easynav::DummyController.");
      return  CallbackReturnT::FAILURE;
    }
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Unable to load plugin easynav::DummyController. Error: %s", ex.what());
    return  CallbackReturnT::FAILURE;
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  controller_main_timer_ = create_timer(1ms, std::bind(&ControllerNode::controller_cycle_rt, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  controller_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
ControllerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

geometry_msgs::msg::TwistStamped
ControllerNode::get_cmd_vel() const
{
  return controller_method_->get_cmd_vel();
}

void
ControllerNode::controller_cycle_rt()
{
  controller_method_->update(nav_state_);
}

void
ControllerNode::controller_cycle_nort()
{
}

}  // namespace easynav
