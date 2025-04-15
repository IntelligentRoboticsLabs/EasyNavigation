// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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
/// \brief Implementation of the EasyNav class.


#include "easynav_core/EasyNav.hpp"

namespace easynav_core
{

EasyNav::EasyNav()
{
  last_perceptions_.resize(PERCEPTION_BUFFER_SIZE);
  valid_perceptions_ = 0;

  configuration_["core.map_type"] = ConfigurationValue("");
}


bool
EasyNav::configure()
{
  return true;
}

std::expected<Speed, std::string>
EasyNav::control_cycle()
{
  // mapper_->update_map(last_perceptions_);
  // localizer_->update_pos(last_perceptions_, mapper_->get_map());
  // planner_->update_path(localizer_->get_pos(), mapper_->get_map(), get_goal());
  // controller_->get_speed(planner_->get_path());

  return Speed();
}

bool
EasyNav::add_perception(Perception & perception)
{
  if (valid_perceptions_ < PERCEPTION_BUFFER_SIZE) {
    last_perceptions_[valid_perceptions_++] = perception;
    return true;
  } else {
    return false;
  }
}


}  // namespace easynav_core
