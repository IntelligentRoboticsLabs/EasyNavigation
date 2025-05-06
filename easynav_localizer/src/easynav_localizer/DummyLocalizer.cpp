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
/// \brief Implementation of the DummyLocalizer class.

#include <expected>
#include "easynav_localizer/DummyLocalizer.hpp"

namespace easynav
{

std::expected<void, std::string> DummyLocalizer::on_initialize()
{
  // Initialize the odometry message
  odom_.header.stamp = get_node()->now();
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";
  odom_.pose.pose.position.x = 0.0;
  odom_.pose.pose.position.y = 0.0;
  odom_.pose.pose.position.z = 0.0;
  odom_.pose.pose.orientation.x = 0.0;
  odom_.pose.pose.orientation.y = 0.0;
  odom_.pose.pose.orientation.z = 0.0;
  odom_.pose.pose.orientation.w = 1.0;

  return {};
}

nav_msgs::msg::Odometry DummyLocalizer::get_odom()
{
  return odom_;
}

void DummyLocalizer::update_rt(const NavState & nav_state)
{
  odom_.header.stamp = nav_state.timestamp;
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";
  // Compute the current robot position...
  // odom_.pose.pose.position.x += 1.0;
}

void DummyLocalizer::update_nort(const NavState & nav_state)
{
  odom_.header.stamp = nav_state.timestamp;
  odom_.header.frame_id = "map";
  odom_.child_frame_id = "base_link";
  // Compute the current robot position...
  // odom_.pose.pose.position.x += 1.0;
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyLocalizer, easynav::LocalizerMethodBase)
