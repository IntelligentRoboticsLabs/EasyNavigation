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
/// \brief Implementation of the MapsManagerNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_maps_manager/MapsManagerNode.hpp"
#include "easynav_core/MapsManagerBase.hpp"

namespace easynav
{

using namespace std::chrono_literals;

MapsManagerNode::MapsManagerNode(const rclcpp::NodeOptions & options)
: LifecycleNode("maps_manager_node", options)
{
  maps_manager_loader_ = std::make_unique<pluginlib::ClassLoader<MapsManagerBase>>(
    "easynav_core", "easynav::MapsManagerBase");

  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
}

MapsManagerNode::~MapsManagerNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  }
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }
}

using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
MapsManagerNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> map_types;
  declare_parameter("map_types", map_types);
  get_parameter("map_types", map_types);

  for (const auto & map_type : map_types) {
    std::string plugin;
    declare_parameter(map_type + std::string(".plugin"), plugin);
    get_parameter(map_type + std::string(".plugin"), plugin);

    try {
      RCLCPP_INFO(get_logger(),
        "Loading MapsManagerBase %s [%s]", map_type.c_str(), plugin.c_str());

      std::shared_ptr<MapsManagerBase> instance;
      instance = maps_manager_loader_->createSharedInstance(plugin);

      auto result = instance->initialize(shared_from_this(), map_type);

      if (!result) {
        RCLCPP_ERROR(get_logger(),
          "Unable to initialize [%s]. Error: %s", plugin.c_str(), result.error().c_str());
        return CallbackReturnT::FAILURE;
      }

      maps_managers_.push_back(instance);

      RCLCPP_INFO(get_logger(),
        "Loaded MapsManagerBase %s [%s]", map_type.c_str(), plugin.c_str());
    } catch (pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(),
        "Unable to load plugin easynav::MapsManagerBase. Error: %s", ex.what());
      return CallbackReturnT::FAILURE;
    }
  }

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  maps_manager_main_timer_ = create_timer(10ms,
    std::bind(&MapsManagerNode::maps_manager_cycle_nort, this), realtime_cbg_);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  maps_manager_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
MapsManagerNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
MapsManagerNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
MapsManagerNode::maps_manager_cycle_rt()
{
}

void
MapsManagerNode::maps_manager_cycle_nort()
{
  for (auto & map_manager : maps_managers_) {
    map_manager->update(nav_state_);
  }
}

}  // namespace easynav
