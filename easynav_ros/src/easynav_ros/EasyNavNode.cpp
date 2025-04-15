// Copyright 2025 Intelligent Robotics Lab
//
// This file is part of the project Easy Navigation (EasyNav in sh0rt)
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
/// \brief Implementation of the EasyNavNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "easynav_ros/EasyNavNode.hpp"
#include "easynav_core/EasyNav.hpp"

namespace easynav_ros
{

using ConfigurationMap = std::map<std::string, easynav_core::ConfigurationValue>;

EasyNavNode::EasyNavNode(const rclcpp::NodeOptions & options)
: LifecycleNode("easynav", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  easynav_core_ = std::make_shared<easynav_core::EasyNav>();

  const ConfigurationMap & config = easynav_core_->get_configuration();

  for (const auto & [key, val] : config) {
    std::visit([&](auto && v) {
        using T = std::decay_t<decltype(v)>;
        declare_parameter<T>(key, v);
    }, val.get_variant());
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
EasyNavNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  bool success = true;

  std::cerr << "Configured" << std::endl;
  ConfigurationMap & config = easynav_core_->get_configuration();

  for (const auto & [key, val] : config) {
    std::visit([&](auto && example_value) {
        using ParamT = std::decay_t<decltype(example_value)>;
        ParamT param_value;
        get_parameter<ParamT>(key, param_value);
        config[key] = param_value;
    }, val.get_variant());
  }

  success = success && easynav_core_->configure();

  if (success) {
    return CallbackReturnT::SUCCESS;
  } else {
    return CallbackReturnT::FAILURE;
  }
}

CallbackReturnT
EasyNavNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
EasyNavNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
EasyNavNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
EasyNavNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
EasyNavNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
EasyNavNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

}  // namespace easynav_ros
