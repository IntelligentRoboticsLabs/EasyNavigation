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
/// \brief Declaration of the EasyNav class and associated data structures for robot navigation.

#ifndef EASYNAV_CORE__EASYNAV_HPP_
#define EASYNAV_CORE__EASYNAV_HPP_

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "tf2/LinearMath/Vector3.hpp"

#include "easynav_core/Configuration.hpp"
#include "easynav_core/Result.hpp"
#include "easynav_core/Mapper.hpp"
#include "easynav_core/Localizer.hpp"
#include "easynav_core/Planner.hpp"
#include "easynav_core/Controller.hpp"

namespace easynav_core
{

/**
 * @struct Perception
 * @brief Contains a sensor observation with timestamp and frame ID.
 *
 * Used for buffering and processing external perception data.
 */
struct Perception
{
  // pcl::PointCloud<pcl::PointXYZ>::Ptr data;
  std::chrono::sys_time<std::chrono::nanoseconds> timestamp;  ///< Timestamp of the perception data.
  std::string frame_id;  ///< Reference frame ID of the perception data.
};

/**
 * @struct Speed
 * @brief Represents linear and angular velocity.
 *
 * Used as the output of the control cycle.
 */
struct Speed
{
  std::chrono::sys_time<std::chrono::nanoseconds> timestamp;  ///< Timestamp of the perception data.
  std::string frame_id;  ///< Reference frame ID of the perception data.
  tf2::Vector3 linear;  ///< Linear velocity.
  tf2::Vector3 angular; ///< Angular velocity.
};

/// @brief Map that stores configuration parameters as type-safe values.
using ConfigurationMap = std::map<std::string, ConfigurationValue>;

/**
 * @class EasyNav
 * @brief Main navigation orchestrator for the Easy Navigation system.
 *
 * This class manages the full navigation pipeline: mapping, localization,
 * planning and control. It acts as the central interface for integrating
 * these modules and running periodic control cycles.
 */
class EasyNav
{
public:
  /// @brief Constructor.
  EasyNav();

  /// @brief Get the internal mapper module.
  /// @return Shared pointer to the mapper.
  std::shared_ptr<Mapper> get_mapper() {return mapper_;}

  /// @brief Get the internal localizer module.
  /// @return Shared pointer to the localizer.
  std::shared_ptr<Localizer> get_localizer() {return localizer_;}

  /// @brief Get the internal planner module.
  /// @return Shared pointer to the planner.
  std::shared_ptr<Planner> get_planner() {return planner_;}

  /// @brief Get the internal controller module.
  /// @return Shared pointer to the controller.
  std::shared_ptr<Controller> get_controller() {return controller_;}

  /// @brief Access the configuration map.
  /// @return Reference to the configuration map.
  ConfigurationMap & get_configuration() {return configuration_;}

  /// @brief Configure the system and its internal modules.
  /// @return true if configuration was successful.
  bool configure();

  /**
   * @brief Executes a full control cycle: localization, planning, control.
   *
   * On success, returns a valid Speed command to be sent to the robot.
   * On failure, returns an error message with diagnostic information.
   *
   * @return Result<Speed, std::string> containing either a valid speed command or an error string.
   */
  Result<Speed, std::string> control_cycle();

  /**
   * @brief Adds a perception entry to the internal buffer.
   * @param perception The new perception to store.
   * @return true if the perception was accepted and stored.
   */
  bool add_perception(Perception & perception);

  /// @brief Maximum number of buffered perceptions.
  static const int PERCEPTION_BUFFER_SIZE = 100;

private:
  std::shared_ptr<Mapper> mapper_;           ///< Pointer to the mapper module.
  std::shared_ptr<Localizer> localizer_;     ///< Pointer to the localizer module.
  std::shared_ptr<Planner> planner_;         ///< Pointer to the planner module.
  std::shared_ptr<Controller> controller_;   ///< Pointer to the controller module.

  ConfigurationMap configuration_;           ///< Stores runtime configuration parameters.

  std::vector<Perception> last_perceptions_; ///< Buffer of recent perception inputs.
  int valid_perceptions_;                    ///< Number of valid perception entries in the buffer.
};

}  // namespace easynav_core

#endif  // EASYNAV_CORE__EASYNAV_HPP_
