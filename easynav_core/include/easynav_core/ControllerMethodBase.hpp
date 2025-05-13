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
/// \brief Declaration of the abstract base class ControllerMethodBase.

#ifndef EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
#define EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_

#include "geometry_msgs/msg/twist_stamped.hpp"

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"

namespace easynav
{

/**
 * @class ControllerMethodBase
 * @brief Abstract base class for control methods in Easy Navigation.
 *
 * This class defines the interface for control algorithm implementations.
 * Derived classes must implement the control logic and provide access to the computed command.
 */
class ControllerMethodBase : public MethodBase
{
public:
  /// @brief Default constructor.
  ControllerMethodBase() = default;

  /// @brief Virtual destructor.
  virtual ~ControllerMethodBase() = default;

  /**
   * @brief Get the current control command.
   *
   * Should return the last control command previously computed by the controller.
   * This function must not invoke the control algorithm itself.
   *
   * @return A TwistStamped message containing the control command.
   */
  [[nodiscard]] virtual geometry_msgs::msg::TwistStamped get_cmd_vel() = 0;

  /**
   * @brief Helper to run the real-time control method if appropriate.
   *
   * Invokes update_rt() only if the method is due or forced by trigger.
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

protected:
  /**
   * @brief Run the control method and update the control command.
   *
   * Called by the system to compute a new control command using the current navigation state.
   *
   * @param nav_state The current state of the navigation system.
   */
  virtual void update_rt(const NavState & nav_state) {}
};

}  // namespace easynav

#endif  // EASYNAV_CORE__CONTROLLERMETHODBASE_HPP_
