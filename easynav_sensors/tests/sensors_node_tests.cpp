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

#include "sensor_msgs/msg/laser_scan.hpp"
#include "easynav_sensors/Perceptions.hpp"

#include "gtest/gtest.h"

class SensorsNodeTestCase : public ::testing::Test
{
protected:
  void SetUp()
  {
    rclcpp::init(0, nullptr);
  }

  void TearDown()
  {
    rclcpp::shutdown();
  }
};

sensor_msgs::msg::LaserScan get_scan_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, std::numeric_limits<float>::infinity());

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 0.0);

  return ret;
}

sensor_msgs::msg::LaserScan get_scan_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 5.0);
  ret.ranges[4] = 0.3;

  return ret;
}


TEST_F(SensorsNodeTestCase, convert_scan2pc)
{
  {
    pcl::PointCloud<pcl::PointXYZ> out;
    easynav_sensors::convert(get_scan_test_1(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);
    for (int i = 0; i < 16; i++) {
      ASSERT_TRUE(std::isnan(out.points[i].x));
      ASSERT_TRUE(std::isnan(out.points[i].y));
      ASSERT_TRUE(std::isnan(out.points[i].z));
    }

    easynav_sensors::convert(get_scan_test_2(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);
    for (int i = 0; i < 16; i++) {
      ASSERT_NEAR(out.points[i].x, 0.0, 0.0001);
      ASSERT_NEAR(out.points[i].y, 0.0, 0.0001);
      ASSERT_NEAR(out.points[i].z, 0.0, 0.0001);
    }

    easynav_sensors::convert(get_scan_test_3(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser");
    ASSERT_EQ(out.width, 16);
    ASSERT_EQ(out.height, 1);
    ASSERT_FALSE(out.is_dense);
    ASSERT_EQ(out.points.size(), 16u);

    ASSERT_NEAR(out.points[0].x, -5.0, 0.0001);
    ASSERT_NEAR(out.points[0].y, 0.0, 0.0001);
    ASSERT_NEAR(out.points[0].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[4].x, 0.0, 0.0001);
    ASSERT_NEAR(out.points[4].y, -0.3, 0.0001);
    ASSERT_NEAR(out.points[4].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[8].x, 5.0, 0.0001);
    ASSERT_NEAR(out.points[8].y, 0.0, 0.0001);
    ASSERT_NEAR(out.points[8].z, 0.0, 0.0001);
    ASSERT_NEAR(out.points[12].x, 0.0, 0.0001);
    ASSERT_NEAR(out.points[12].y, 5.0, 0.0001);
    ASSERT_NEAR(out.points[12].z, 0.0, 0.0001);
  }

}
