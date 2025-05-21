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
#include "easynav_common/YTSession.hpp"

namespace easynav
{

using namespace std::chrono_literals;

ControllerNode::ControllerNode(
  const std::shared_ptr<const NavState> & nav_state,
  const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options),
  nav_state_(nav_state)
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

  std::vector<std::string> controller_types;
  get_parameter("controller_types", controller_types);
  for (const auto & controller_type : controller_types) {
    controller_loader_->unloadLibraryForClass(controller_type);
  }
  controller_method_ = nullptr;
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ControllerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> controller_types;
  declare_parameter("controller_types", controller_types);
  get_parameter("controller_types", controller_types);

  if (controller_types.size() > 1) {
    RCLCPP_ERROR(get_logger(),
      "You must instance one controller.  [%lu] found", controller_types.size());
    return CallbackReturnT::FAILURE;
  }

  for (const auto & controller_type : controller_types) {
    std::string plugin;
    declare_parameter(controller_type + std::string(".plugin"), plugin);
    get_parameter(controller_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading ControllerMethodBase %s [%s]", controller_type.c_str(), plugin.c_str());

      controller_method_ = controller_loader_->createSharedInstance(plugin);

      auto result = controller_method_->initialize(shared_from_this(), controller_type);

      if (!result) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
        return CallbackReturnT::FAILURE;
      }

      RCLCPP_INFO(get_logger(),
        "Loaded ControllerMethodBase %s [%s]", controller_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin easynav::ControllerMethodBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

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
  if (controller_method_ == nullptr) {
    return geometry_msgs::msg::TwistStamped();
  }

  return controller_method_->get_cmd_vel();
}

bool
ControllerNode::cycle_rt(bool trigger)
{
  EASYNAV_TRACE_EVENT;

  if (controller_method_ == nullptr) {return false;}

  return controller_method_->internal_update_rt(*nav_state_, trigger);
}

}  // namespace easynav
