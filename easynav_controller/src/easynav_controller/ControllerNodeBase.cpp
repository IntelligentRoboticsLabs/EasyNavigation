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
/// \brief Implementation of the ControllerNodeBase class.

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_controller/ControllerNodeBase.hpp"

namespace easynav_controller
{

using namespace std::chrono_literals;

ControllerNodeBase::ControllerNodeBase(const rclcpp::NodeOptions & options)
: LifecycleNode("controller_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
ControllerNodeBase::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNodeBase::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  controller_main_timer_ = create_timer(1ms, std::bind(&ControllerNodeBase::controller_cycle, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNodeBase::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  controller_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNodeBase::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNodeBase::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
ControllerNodeBase::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
ControllerNodeBase::get_real_time_cbg()
{
  return realtime_cbg_;
}

geometry_msgs::msg::TwistStamped ControllerNodeBase::get_cmd_vel() const
{
  return cmd_vel_;
}

}  // namespace easynav_controller
