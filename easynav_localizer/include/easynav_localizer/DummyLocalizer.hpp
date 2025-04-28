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
/// \brief Declaration of the DummyLocalizer method.

#ifndef EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_

#include "nav_msgs/msg/odometry.hpp"

#include "easynav_core/LocalizerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyLocalizer
 * @brief A default "dummy" implementation for the Localizer Method.
 *
 * This localization method does nothing. It serves as an example, and will be used as a default plugin implementation
 * if the navigation system configuration does not specify one.
 *
 */
class DummyLocalizer : public easynav::LocalizerMethodBase
{
public:
  DummyLocalizer() = default;
  ~DummyLocalizer() = default;

  /**
   * @brief Initialize the localization method.
   *
   * It is not required to override this method. Only if the derived class
   * requires further initialization than the provided by the base class.
   */
  virtual void on_initialize() override;

  /**
   * @brief Get the current localization state.
   *
   * This method should return the last known localization of the robot.
   * It should not run the localization algorithm (see update method).
   *
   * @return An Odometry message representing the current localization state.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() override;

  /**
   * @brief Run the localization method and update the robot's estimated localization.
   *
   * This method will be called by the system's LocalizerNode to run the localization algorithm.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /**
   * @brief Current robot localization state.
   */
  nav_msgs::msg::Odometry odom_ {};
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
