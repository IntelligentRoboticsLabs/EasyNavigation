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
/// \brief Implementation of the SensorsNode class.

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "easynav_sensors/SensorsNode.hpp"

namespace easynav_sensors
{

using namespace std::chrono_literals;

SensorsNode::SensorsNode(const rclcpp::NodeOptions & options)
: LifecycleNode("sensors_node", options)
{
  realtime_cbg_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  percept_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "sensors_node/perceptions", rclcpp::SensorDataQoS().reliable());

  if (!has_parameter("sensors")) {
    declare_parameter("sensors", std::vector<std::string>());
  }

  if (!has_parameter("forget_time")) {
    declare_parameter("forget_time", 1.0);
  }

  if (!has_parameter("perception_default_frame")) {
    perception_default_frame_ = "odom";
    declare_parameter("perception_default_frame", perception_default_frame_);
  }

}

SensorsNode::~SensorsNode()
{
  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }
}


using CallbackReturnT = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

CallbackReturnT
SensorsNode::on_configure(const rclcpp_lifecycle::State & state)
{
  (void)state;

  std::vector<std::string> sensors;
  get_parameter("sensors", sensors);
  get_parameter("forget_time", forget_time_);
  get_parameter("perception_default_frame", perception_default_frame_);

  for (const auto & sensor : sensors) {
    std::string topic, msg_type;

    if (!has_parameter(sensor + ".topic")) {
      declare_parameter(sensor + ".topic", topic);
    }
    if (!has_parameter(sensor + ".type")) {
      declare_parameter(sensor + ".type", msg_type);
    }

    get_parameter(sensor + ".topic", topic);
    get_parameter(sensor + ".type", msg_type);

    auto perception_entry = std::make_shared<Perception>();
    perception_entry->data.points.clear();
    perception_entry->data.clear();
    perception_entry->frame_id = "";
    perception_entry->stamp = now();
    perception_entry->valid = false;

    perceptions_.push_back(perception_entry);

    if (msg_type == "LaserScan") {
      perception_entry->subscription = create_typed_subscription<sensor_msgs::msg::LaserScan>(
        *this, topic, perception_entry, realtime_cbg_);
    } else if (msg_type == "PointCloud") {
      perception_entry->subscription = create_typed_subscription<sensor_msgs::msg::PointCloud2>(
        *this, topic, perception_entry, realtime_cbg_);
    } else {
      RCLCPP_ERROR(get_logger(), "Sensor type [%s] not supported", msg_type.c_str());
      return CallbackReturnT::FAILURE;
    }
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, shared_from_this(),
    true);

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_activate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  sensors_main_timer_ = create_timer(100ms, std::bind(&SensorsNode::sensors_cycle, this));

  percept_pub_->on_activate();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  (void)state;

  percept_pub_->on_deactivate();
  sensors_main_timer_->cancel();

  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

CallbackReturnT
SensorsNode::on_error(const rclcpp_lifecycle::State & state)
{
  (void)state;
  return CallbackReturnT::SUCCESS;
}

rclcpp::CallbackGroup::SharedPtr
SensorsNode::get_real_time_cbg()
{
  return realtime_cbg_;
}

void
SensorsNode::sensors_cycle()
{
  for (auto & perception : perceptions_) {
    if (perception->valid && (now() - perception->stamp).seconds() > forget_time_) {
      perception->valid = false;
    }
  }

  if (percept_pub_->get_subscription_count() > 0) {
    fuse_perceptions(perceptions_, perception_default_frame_, *tf_buffer_, perecption_msg_);
    percept_pub_->publish(perecption_msg_);
  }
}


}  // namespace easynav_sensors
