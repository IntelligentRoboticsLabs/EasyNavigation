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
/// \brief Declaration of the abstract base class LocalizerMethodBase.

#ifndef EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
#define EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class LocalizerMethodBase
 * @brief Abstract base class for localization methods in Easy Navigation.
 *
 * This class defines the interface for localization algorithm implementations.
 * Derived classes must implement the update and get_odom methods.
 */
class LocalizerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  LocalizerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~LocalizerMethodBase() = default;

  /**
   * @brief Get the current localization state.
   *
   * Should return the last estimated pose of the robot.
   *
   * @return An Odometry message representing the current estimated state.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() = 0;

  /**
   * @brief Helper to run the real-time update if appropriate.
   *
   * @param nav_state The current state of the navigation system.
   * @param trigger Force execution regardless of timing.
   * @return True if update_rt() was called, false otherwise.
   */
  bool internal_update_rt(const NavState & nav_state, bool trigger = false)
  {
    if (isTime2RunRT() || trigger) {
      update_rt(nav_state);
      return true;
    } else {
      return false;
    }
  }

  /**
   * @brief Helper to run the non-real-time update if appropriate.
   *
   * @param nav_state The current state of the navigation system.
   */
  void internal_update(const NavState & nav_state)
  {
    if (isTime2Run()) {
      update(nav_state);
    }
  }

protected:
  /**
   * @brief Run the real-time localization update.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt(const NavState & nav_state) = 0;

  /**
   * @brief Run the non-real-time localization update.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) = 0;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__LOCALIZERMETHODBASE_HPP_
