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
/// \brief Declaration of the base class MethodBase used in plugin-based EasyNav modules.

#ifndef EASYNAV_CORE__METHODBASE_HPP_
#define EASYNAV_CORE__METHODBASE_HPP_

#include <memory>
#include <expected>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class MethodBase
 * @brief Base class for Easy Navigation method plugins.
 *
 * This abstract base class provides common functionality for all plugin-based
 * modules in Easy Navigation, including access to the parent lifecycle node
 * and the plugin name.
 */
class MethodBase
{
public:
  /**
   * @brief Default constructor for MethodBase.
   */
  MethodBase() = default;

  /**
   * @brief Virtual destructor for MethodBase.
   *
   * Ensures proper destruction of derived classes.
   */
  virtual ~MethodBase() = default;

  /**
   * @brief Initializes the module method with the given parent node and plugin name.
   *
   * This method should be called to set up any necessary resources or configuration
   * required by the module. By default, it stores the provided lifecycle node and
   * plugin name, and delegates additional initialization to the virtual on_initialize() method.
   *
   * @param parent_node Shared pointer to the parent lifecycle node.
   * @param plugin_name Name of the plugin (used for logging or namespacing).
   * @return std::expected<void, std::string> Returns:
   *         - `void` if initialization succeeds,
   *         - a `std::string` containing the error message if it fails.
   *
   * @note If this method is overridden in a derived class, it should explicitly call
   *       the base class implementation to ensure proper setup of the parent node and plugin name.
   */
  virtual std::expected<void, std::string>
  initialize(
    const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node,
    const std::string plugin_name)
  {
    parent_node_ = parent_node;
    plugin_name_ = plugin_name;

    rt_frequency_ = 10.0;
    frequency_ = 10.0;

    parent_node_->declare_parameter(plugin_name + ".rt_freq", rt_frequency_);
    parent_node_->declare_parameter(plugin_name + ".freq", frequency_);
    parent_node_->get_parameter(plugin_name + ".rt_freq", rt_frequency_);
    parent_node_->get_parameter(plugin_name + ".freq", frequency_);

    last_ts_ = parent_node_->now();
    rt_last_ts_ = parent_node_->now();

    return on_initialize();
  }

  /**
   * @brief Called during initialization for custom setup by the derived class.
   *
   * This method is automatically called by initialize(), and is intended to be overridden
   * by derived plugins if they need to perform additional setup or resource allocation.
   *
   * @return std::expected<void, std::string> Returns:
   *         - `void` if successful,
   *         - a `std::string` with the error message on failure.
   */
  virtual std::expected<void, std::string> on_initialize() {return {};}

  /**
   * @brief Get a shared pointer to the parent lifecycle node.
   *
   * Useful for accessing parameters, logging, and other ROS interfaces in derived classes.
   *
   * @return A shared pointer to the parent rclcpp_lifecycle::LifecycleNode.
   */
  [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
  get_node() const
  {
    return parent_node_;
  }

  /**
   * @brief Get the plugin name assigned during initialization.
   *
   * Typically used for namespacing, logging or plugin-specific configuration.
   *
   * @return A const reference to the plugin name string.
   */
  [[nodiscard]] const std::string &
  get_plugin_name() const
  {
    return plugin_name_;
  }

  bool isTime2RunRT()
  {
    if ((parent_node_->now() - rt_last_ts_).seconds() > (1.0 / rt_frequency_)) {
      rt_last_ts_ = parent_node_->now();
      return true;
    } else {
      return false;
    }
  }

  bool isTime2Run()
  {
    if ((parent_node_->now() - last_ts_).seconds() > (1.0 / frequency_)) {
      last_ts_ = parent_node_->now();
      return true;
    } else {
      return false;
    }
  }

private:
  /**
   * @brief Pointer to the parent lifecycle node.
   *
   * Provides access to parameters, logging, and other ROS-related functionality.
   */
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};

  /**
   * @brief Name assigned to the plugin during initialization.
   *
   * Used for identification, namespacing, and logging.
   */
  std::string plugin_name_;

  float rt_frequency_, frequency_;
  rclcpp::Time rt_last_ts_, last_ts_;
};

}  // namespace easynav

#endif  // EASYNAV_CORE__METHODBASE_HPP_
