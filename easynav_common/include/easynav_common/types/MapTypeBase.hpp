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
/// \brief Declaration of the MapsTypeBase template class, an abstract interface to represent map types.

#ifndef EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_
#define EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_

#include "easynav_common/types/Perceptions.hpp"

namespace easynav
{

/**
 * @class MapsTypeBase
 * @brief Abstract base class for managing map data in Easy Navigation.
 *
 * This class defines an interface for modules responsible for maintaining
 * static and dynamic maps, including publishing and updating based on incoming perceptions.
 *
 */

class MapsTypeBase
{
public:
  /**
   * @brief Construct a new MapsTypeBase
   */
  MapsTypeBase() = default;

  /**
   * @brief Virtual destructor for MapsTypeBase
   */
  virtual ~MapsTypeBase() = default;

};

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__MAPSTYPEBASE_HPP_
