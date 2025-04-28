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
/// \brief Implementation of the DummyPlanner class.

#include "easynav_planner/DummyPlanner.hpp"

namespace easynav
{

void DummyPlanner::on_initialize()
{
  // Initialize the Path message
  path_.header.stamp = get_node()->now();
  path_.header.frame_id = "map";
  path_.poses.clear();
}

nav_msgs::msg::Path DummyPlanner::get_path()
{
  return path_;
}

void DummyPlanner::update(const NavState & nav_state)
{
  path_.header.stamp = nav_state.timestamp;
  path_.header.frame_id = "map";
  // Compute the current path...
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyPlanner, easynav::PlannerMethodBase)
