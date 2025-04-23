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
/// \brief Implementation of the LocalizerNodeBase class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_localizer/LocalizerNodeBase.hpp"

namespace easynav_localizer
{

using namespace std::chrono_literals;

LocalizerNodeBase::LocalizerNodeBase(const rclcpp::NodeOptions & options)
: LifecycleNode("localizer_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
LocalizerNodeBase::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNodeBase::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  localizer_main_timer_ = create_timer(1ms, std::bind(&LocalizerNodeBase::localizer_cycle, this),
    realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNodeBase::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  localizer_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNodeBase::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNodeBase::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
LocalizerNodeBase::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
LocalizerNodeBase::get_real_time_cbg()
{
  return realtime_cbg_;
}

nav_msgs::msg::Odometry LocalizerNodeBase::get_pose() const
{
  return robot_odom_;
}

}  // namespace easynav_localizer
