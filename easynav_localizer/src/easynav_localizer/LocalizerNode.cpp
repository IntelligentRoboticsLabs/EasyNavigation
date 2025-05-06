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
/// \brief Implementation of the LocalizerNode class.

#include "pluginlib/class_loader.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_localizer/LocalizerNode.hpp"

namespace easynav
{

using namespace std::chrono_literals;

LocalizerNode::LocalizerNode(
    const std::shared_ptr<const NavState> & nav_state,
    const rclcpp::NodeOptions & options)
: LifecycleNode("localizer_node", options),
  nav_state_(nav_state)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  localizer_loader_ = std::make_unique<pluginlib::ClassLoader<easynav::LocalizerMethodBase>>(
    "easynav_core", "easynav::LocalizerMethodBase");
}

LocalizerNode::~LocalizerNode()
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
LocalizerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  try {
    localizer_method_ = localizer_loader_->createSharedInstance("easynav_localizer/DummyLocalizer");
    auto result = localizer_method_->initialize(shared_from_this(), "dummy_localizer");
    if (!result) {
      RCLCPP_ERROR(get_logger(),
        "Unable to initialize [dummy_localizer]. Error: %s", result.error().c_str());
      return CallbackReturnT::FAILURE;
    }
  } catch (pluginlib::PluginlibException & ex) {
    RCLCPP_ERROR(get_logger(),
      "Unable to load plugin easynav::DummyLocalizer. Error: %s", ex.what());
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  localizer_main_timer_ = create_timer(1ms, std::bind(&LocalizerNode::localizer_cycle_nort, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  localizer_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
LocalizerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

nav_msgs::msg::Odometry
LocalizerNode::get_odom() const
{
  return localizer_method_->get_odom();
}

bool
LocalizerNode::localizer_cycle_rt(bool trigger)
{
  return localizer_method_->internal_update_rt(nav_state_, trigger);
}

void
LocalizerNode::localizer_cycle_nort()
{
  localizer_method_->update_nort(nav_state_);
}

}  // namespace easynav
