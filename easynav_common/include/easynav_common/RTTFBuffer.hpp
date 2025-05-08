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


#ifndef EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
#define EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_

#include "tf2_ros/buffer.h"
#include "rclcpp/rclcpp.hpp"

namespace easynav
{

class RTTFBuffer {
public:
  static std::shared_ptr<tf2_ros::Buffer> getInstance(const rclcpp::Clock::SharedPtr & clock)
  {
    if (!instance_) {
      instance_ = std::make_shared<tf2_ros::Buffer>(clock);
    }
    return instance_;
  }

  static std::shared_ptr<tf2_ros::Buffer> getInstance()
  {
    if (!instance_) {
      throw std::runtime_error("RTTFBuffer not initialized");
    }
    return instance_;
  }

  static void removeInstance()
  {
    instance_.reset();
  }

private:
  static std::shared_ptr<tf2_ros::Buffer> instance_;
};

std::shared_ptr<tf2_ros::Buffer> RTTFBuffer::instance_ = nullptr;

}  // namespace easynav

#endif  // EASYNAV_COMMON_TYPES__RTTFBUFFER_HPP_
