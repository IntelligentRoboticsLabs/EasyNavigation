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
/// \brief Declaration of the DummyPlanner class, a default fallback planner plugin.

#ifndef EASYNAV_PLANNER__DUMMYPLANNER_HPP_
#define EASYNAV_PLANNER__DUMMYPLANNER_HPP_

#include <expected>

#include "nav_msgs/msg/path.hpp"
#include "easynav_core/PlannerMethodBase.hpp"

namespace easynav
{

/**
 * @class DummyPlanner
 * @brief A default "do-nothing" implementation of PlannerMethodBase.
 *
 * Used as a fallback when no real planner is configured.
 */
class DummyPlanner : public easynav::PlannerMethodBase
{
public:
  /// @brief Default constructor.
  DummyPlanner() = default;

  /// @brief Destructor.
  ~DummyPlanner() = default;

  /**
   * @brief Initialization hook.
   * @return Success or error message.
   */
  virtual std::expected<void, std::string> on_initialize() override;

  /**
   * @brief Get the current path.
   * @return Stored (empty) path.
   */
  [[nodiscard]] virtual nav_msgs::msg::Path get_path() override;

  /**
   * @brief Dummy update method.
   * @param nav_state Current navigation state.
   */
  virtual void update(const NavState & nav_state) override;

private:
  /// @brief Stored path message (unused in dummy).
  nav_msgs::msg::Path path_ {};
};

}  // namespace easynav

#endif  // EASYNAV_PLANNER__DUMMYPLANNER_HPP_
