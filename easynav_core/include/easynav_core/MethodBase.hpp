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
/// \brief Declaration of the base class MethodBase.

#ifndef EASYNAV_CORE__METHODBASE_HPP_
#define EASYNAV_CORE__METHODBASE_HPP_

#include <memory>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace easynav
{

/**
 * @class MethodBase
 * @brief Base class for abstract methods in Easy Navigation.
 *
 * This class provides the common functionality that all methods have,
 * like managing the parent Lifecycle Node.
 *
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
   * This ensures proper cleanup of derived classes.
   */
  virtual ~MethodBase() = default;

  /**
   * @brief Initialize the module method.
   *
   * This method should be called to set up any necessary resources
   * or configurations for the algorithm.
   * By default, it sets the parent node to the provided lifecycle node.
   *
   * @note If this method is overriden, the derived class should call the base method
   * to ensure proper initialization of the parent node.
   */
  virtual void
  initialize(const std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node)
  {
    parent_node_ = parent_node;
    on_initialize();
  }

  /**
   * @brief Initialize the method.
   *
   * This method can be overriden to set up any necessary resources
   * or configurations for the derived algorithm.
   * It will be called after the base initialize() ends.
   *
   */
  virtual void on_initialize() {}

  /**
   * @brief get a pointer to the parent lifecycle node.
   *
   * This method can be used by derived classes to access the (private) lifecycle node.
   *
   * @return A shared pointer to the parent lifecycle node.
   */
  [[nodiscard]] std::shared_ptr<rclcpp_lifecycle::LifecycleNode>
  get_node() const
  {
    return parent_node_;
  }

private:
  /**
   * @brief Pointer to the parent lifecycle node.
   *
   * This is used to access ROS interfaces and parameters.
   */
  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> parent_node_ {nullptr};
};

}  // namespace easynav

#endif  // EASYNAV_CORE__METHODBASE_HPP_
