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


#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_system/SystemNode.hpp"

#include "easynav_common/RTTFBuffer.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exe_nort, exe_rt;
  auto system_node = easynav::SystemNode::make_shared();

  exe_nort.add_node(system_node->get_node_base_interface());
  exe_rt.add_callback_group(system_node->get_real_time_cbg(),
    system_node->get_node_base_interface());

  auto tf_node = rclcpp::Node::make_shared("tf_node");
  exe_rt.add_node(tf_node);

  auto tf_buffer = easynav::RTTFBuffer::getInstance(tf_node->get_clock());

  for (auto & node : system_node->get_system_nodes()) {
    exe_nort.add_node(node.second.node_ptr->get_node_base_interface());
    if (node.second.realtime_cbg != nullptr) {
      exe_rt.add_callback_group(node.second.realtime_cbg,
        node.second.node_ptr->get_node_base_interface());
    }
  }

  system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (system_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(system_node->get_logger(), "Unable to configure EasyNav");
    rclcpp::shutdown();
    return 1;
  }
  system_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (system_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(system_node->get_logger(), "Unable to activate EasyNav");
    rclcpp::shutdown();
    return 1;
  }

  auto rt_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 80;

      if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
        RCLCPP_ERROR(
          system_node->get_logger(),
          "Failed to tet EasyNav to execute in Real Time.");
        RCLCPP_ERROR(system_node->get_logger(), "set your system to have permissions.");
        throw std::runtime_error{std::string("failed to set sched: ") + std::strerror(errno)};
      }

      tf2_ros::TransformListener tf_listener(*tf_buffer, tf_node, true);

      rclcpp::Rate rate(100);
      while (rclcpp::ok()) {
        if (system_node->get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
          system_node->system_cycle_rt();
        }
        rate.sleep();
        exe_rt.spin_some();
      }
    });

  exe_nort.spin();

  rt_thread.join();

  // TODO: There is an issue here to cleanly finish https://github.com/ros2/rclcpp/issues/2520

  rclcpp::shutdown();
  return 0;
}
