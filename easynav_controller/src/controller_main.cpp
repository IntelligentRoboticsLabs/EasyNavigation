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


#include <string>

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "easynav_controller/ControllerNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::experimental::executors::EventsExecutor exe_nort, exe_rt;

  auto controller_node = easynav_controller::ControllerNode::make_shared();

  exe_nort.add_node(controller_node->get_node_base_interface());
  exe_rt.add_callback_group(controller_node->get_real_time_cbg(),
    controller_node->get_node_base_interface());

  controller_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
  if (controller_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    RCLCPP_ERROR(controller_node->get_logger(), "Unable to configure EasyNav");
    rclcpp::shutdown();
    return 1;
  }
  controller_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  if (controller_node->get_current_state().id() !=
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    RCLCPP_ERROR(controller_node->get_logger(), "Unable to activate EasyNav");
    rclcpp::shutdown();
    return 1;
  }

  auto rt_thread = std::thread(
    [&]() {
      sched_param sch;
      sch.sched_priority = 80;

      if (sched_setscheduler(0, SCHED_FIFO, &sch) == -1) {
        RCLCPP_ERROR(
          controller_node->get_logger(),
          "Failed to tet EasyNav to execute in Real Time.");
        RCLCPP_ERROR(controller_node->get_logger(), "set your system to have permissions.");
        throw std::runtime_error{std::string("failed to set sched: ") + std::strerror(errno)};
      }

      exe_rt.spin();
    });

  exe_nort.spin();

  rt_thread.join();

  // TODO: There is an issue here to cleanly finish https://github.com/ros2/rclcpp/issues/2520

  rclcpp::shutdown();
  return 0;
}
