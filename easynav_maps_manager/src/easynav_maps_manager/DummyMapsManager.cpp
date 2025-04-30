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
/// \brief Implementation of the DummyMapsManager class.

#include <expected>

#include "easynav_maps_manager/DummyMapsManager.hpp"

namespace easynav
{

std::expected<void, std::string> DummyMapsManager::on_initialize()
{
  return {};
}

std::shared_ptr<MapsTypeBase>
DummyMapsManager::get_static_map()
{
  return nullptr;
}

std::shared_ptr<MapsTypeBase>
DummyMapsManager::get_dynamyc_map()
{
  return nullptr;
}

void
DummyMapsManager::update(const NavState & nav_state)
{
}

}  // namespace easynav

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(easynav::DummyMapsManager, easynav::MapsManagerBase)
