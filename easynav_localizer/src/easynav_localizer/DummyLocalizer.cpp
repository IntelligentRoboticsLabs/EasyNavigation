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

#include "easynav_common/RTTFBuffer.hpp"

namespace easynav
{

std::expected<void, std::string> DummyLocalizer::on_initialize()
{
  auto node = get_node();
  const auto & plugin_name = get_plugin_name();

  node->declare_parameter<double>(plugin_name + ".cycle_time_rt", 0.01);
  node->declare_parameter<double>(plugin_name + ".cycle_time_nort", 0.01);
  node->get_parameter<double>(plugin_name + ".cycle_time_rt", cycle_time_rt_);
  node->get_parameter<double>(plugin_name + ".cycle_time_nort", cycle_time_nort_);

  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(get_node());

  return {};
}

nav_msgs::msg::Odometry DummyLocalizer::get_odom()
{
  return odom_;
}

void DummyLocalizer::update_rt(const NavState & nav_state)
{
  auto start = get_node()->now();
  while ((get_node()->now() - start).seconds() < cycle_time_rt_) {}

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = get_node()->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);
}

void DummyLocalizer::update(const NavState & nav_state)
{
  auto start = get_node()->now();
  while ((get_node()->now() - start).seconds() < cycle_time_nort_) {}

  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = get_node()->now();
  tf_msg.header.frame_id = "map";
  tf_msg.child_frame_id = "odom";

  RTTFBuffer::getInstance()->setTransform(tf_msg, "easynav", false);
  tf_broadcaster_->sendTransform(tf_msg);
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyLocalizer, easynav::LocalizerMethodBase)
