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
/// \brief Declaration of the DummyLocalizer class, a default plugin implementation for localization.

#ifndef EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
#define EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_

#include <expected>

#include "nav_msgs/msg/odometry.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace easynav
{

/**
 * @class DummyLocalizer
 * @brief A default "do-nothing" implementation of LocalizerMethodBase.
 *
 * This class complies with the localization interface but does not perform real computation.
 * It is intended as a placeholder, fallback, or example plugin.
 */
class DummyLocalizer : public easynav::LocalizerMethodBase
{
public:
  /// @brief Default constructor.
  DummyLocalizer() = default;

  /// @brief Default destructor.
  ~DummyLocalizer() = default;

  /**
   * @brief Plugin-specific initialization logic.
   *
   * @return Success or an error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Get the last stored odometry estimate.
   *
   * @return Odometry message with current localization estimate.
   */
  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() override;

  /**
   * @brief Update the localization using the current navigation state.
   *
   * This dummy version performs no actual computation.
   *
   * @param nav_state The current navigation state.
   */
  virtual void update_rt(const NavState & nav_state) override;

  /**
   * @brief Update the localization using the current navigation state.
   *
   * This dummy version performs no actual computation.
   *
   * @param nav_state The current navigation state.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /// @brief Internal odometry placeholder.
  nav_msgs::msg::Odometry odom_ {};

  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double cycle_time_rt_;
  double cycle_time_nort_;
};

}  // namespace easynav

#endif  // EASYNAV_LOCALIZER__DUMMYLOCALIZER_HPP_
