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
/// \brief Implementation of the DummyController class.

#include <expected>

#include "easynav_controller/DummyController.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

namespace easynav
{

std::expected<void, std::string> DummyController::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_rt", 0.01);
  node->declare_parameter<double>(plugin_name + ".cycle_time_nort", 0.01);
  node->get_parameter<double>(plugin_name + ".cycle_time_rt", cycle_time_rt_);
  node->get_parameter<double>(plugin_name + ".cycle_time_nort", cycle_time_nort_);

  // Initialize the odometry message
  cmd_vel_.header.stamp = get_node()->now();
  cmd_vel_.header.frame_id = "base_link";
  cmd_vel_.twist.linear.x = 0.0;
  cmd_vel_.twist.linear.y = 0.0;
  cmd_vel_.twist.linear.z = 0.0;
  cmd_vel_.twist.angular.x = 0.0;
  cmd_vel_.twist.angular.y = 0.0;
  cmd_vel_.twist.angular.z = 0.0;

  return {};
}

geometry_msgs::msg::TwistStamped DummyController::get_cmd_vel()
{
  return cmd_vel_;
}

void DummyController::update_rt(const NavState & nav_state)
{
  auto start = get_node()->now();
  while ((get_node()->now() - start).seconds() < cycle_time_rt_) {}

  cmd_vel_.header.stamp = nav_state.timestamp;
  cmd_vel_.header.frame_id = "base_link";
  // Compute the current command...
  // cmd_vel_.angular.z = 1.0;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyController, easynav::ControllerMethodBase)
