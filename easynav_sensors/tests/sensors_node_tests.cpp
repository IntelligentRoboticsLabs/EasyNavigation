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
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "easynav_common/types/Perceptions.hpp"
#include "easynav_sensors/SensorsNode.hpp"
#include "easynav_common/RTTFBuffer.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_types_conversion.h"
#include "pcl/common/transforms.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/transform_datatypes.h"

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
  ret.header.frame_id = "base_laser_1";
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

sensor_msgs::msg::LaserScan get_scan_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::LaserScan ret;
  ret.header.frame_id = "base_laser_2";
  ret.header.stamp = ts;
  ret.angle_min = -M_PI;
  ret.angle_max = M_PI;
  ret.range_min = 0.0;
  ret.range_max = 100.0;
  ret.angle_increment = 2.0 * M_PI / 16.0;
  ret.ranges = std::vector<float>(16, 4.0);
  ret.ranges[4] = 1.0;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_0(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = static_cast<float>(i);
    p.y = static_cast<float>(i);
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_1(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = std::numeric_limits<float>::infinity();
    p.y = std::numeric_limits<float>::infinity();
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_2(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 0.0;
    p.y = 0.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_3(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 0.3;
  pc[4].y = 0.3;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

sensor_msgs::msg::PointCloud2 get_pc2_test_4(rclcpp::Time ts)
{
  sensor_msgs::msg::PointCloud2 ret;
  pcl::PointCloud<pcl::PointXYZ> pc;

  pc.header.frame_id = "base_lidar3d";
  pc.width = 16;
  pc.height = 1;
  pc.is_dense = false;

  for (int i = 0; i < 16; i++) {
    pcl::PointXYZ p;
    p.x = 5.0;
    p.y = 5.0;
    p.z = 0.0;
    pc.push_back(p);
  }

  pc[4].x = 1.0;
  pc[4].y = 1.0;
  pc[4].z = 0.0;

  pcl::toROSMsg(pc, ret);
  ret.header.frame_id = pc.header.frame_id;
  ret.header.stamp = ts;

  return ret;
}

using namespace std::chrono_literals;

TEST_F(SensorsNodeTestCase, convert_scan2pc)
{
  {
    pcl::PointCloud<pcl::PointXYZ> out;
    easynav::convert(get_scan_test_1(rclcpp::Time()), out);

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

    easynav::convert(get_scan_test_2(rclcpp::Time()), out);

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

    easynav::convert(get_scan_test_3(rclcpp::Time()), out);

    ASSERT_EQ(out.header.frame_id, "base_laser_1");
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

TEST_F(SensorsNodeTestCase, percept_laserscan)
{
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("laser1.type", std::string("LaserScan"));
  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("LaserScan")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto ts = test_node->now();
  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      ts = test_node->now();
      laser_pub->publish(get_scan_test_1(ts));
      exe.spin_some();
    }
  }

  const auto & perceptions = sensors_node->get_perceptions();

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
  ASSERT_EQ(perceptions[0]->frame_id, "base_laser");
  ASSERT_EQ(perceptions[0]->valid, true);
  ASSERT_NE(perceptions[0]->subscription, nullptr);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      exe.spin_some();
    }
  }

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_EQ(perceptions[0]->frame_id, "base_laser");
  ASSERT_EQ(perceptions[0]->valid, false);
  ASSERT_NE(perceptions[0]->subscription, nullptr);
}

TEST_F(SensorsNodeTestCase, percept_fuse_laserscan)
{
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser1_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());
  auto laser2_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan2", rclcpp::SensorDataQoS().reliable());

  sensor_msgs::msg::PointCloud2::SharedPtr fused_perception;
  auto fused_percept_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensors_node/perceptions", rclcpp::SensorDataQoS().reliable(),
    [&fused_perception](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      fused_perception = msg;
    });

  easynav::RTTFBuffer::getInstance(test_node->get_clock());

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*test_node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 1.0;

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1", "laser2"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("laser1.type", std::string("LaserScan"));
  sensors_node->declare_parameter("laser2.topic", std::string("/scan2"));
  sensors_node->declare_parameter("laser2.type", std::string("LaserScan"));
  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("LaserScan")});
  sensors_node->set_parameter({"laser2.topic", std::string("/scan2")});
  sensors_node->set_parameter({"laser2.type", std::string("LaserScan")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
  
      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      laser2_pub->publish(get_scan_test_4(time2));
      exe.spin_some();
    }

    const auto & perceptions = sensors_node->get_perceptions();

    ASSERT_EQ(perceptions.size(), 2u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[1]->stamp).seconds(), 0.0, 0.02);
    ASSERT_EQ(perceptions[1]->valid, true);
    ASSERT_NE(perceptions[1]->subscription, nullptr);
    ASSERT_LT(perceptions[1]->stamp, perceptions[0]->stamp);

    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 32u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";

      easynav::RTTFBuffer::getInstance()->setTransform(transform, "easynav", false);
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      exe.spin_some();
    }

    const auto & perceptions = sensors_node->get_perceptions();

    ASSERT_EQ(perceptions.size(), 2u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_EQ(perceptions[1]->valid, false);
    ASSERT_NE(perceptions[1]->subscription, nullptr);

    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 16u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }

}

TEST_F(SensorsNodeTestCase, percept_pc2)
{
  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser3d_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc1", rclcpp::SensorDataQoS().reliable());

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser3d1"};
  sensors_node->declare_parameter("laser3d1.topic", std::string("/pc1"));
  sensors_node->declare_parameter("laser3d1.type", std::string("PointCloud"));
  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser3d1.topic", std::string("/pc1")});
  sensors_node->set_parameter({"laser3d1.type", std::string("PointCloud")});

  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  auto ts = test_node->now();
  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      ts = test_node->now();
      laser3d_pub->publish(get_pc2_test_0(ts));
      exe.spin_some();
    }
  }

  const auto & perceptions = sensors_node->get_perceptions();

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
  ASSERT_EQ(perceptions[0]->frame_id, "base_lidar3d");
  ASSERT_EQ(perceptions[0]->valid, true);
  ASSERT_NE(perceptions[0]->subscription, nullptr);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      exe.spin_some();
    }
  }

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 16u);
  ASSERT_EQ(perceptions[0]->frame_id, "base_lidar3d");
  ASSERT_EQ(perceptions[0]->valid, false);
  ASSERT_NE(perceptions[0]->subscription, nullptr);
}


/*
TEST(SensorsNodeTestCase, percept_fuse_all)
{
  rclcpp::init(0, nullptr);

  auto sensors_node = easynav::SensorsNode::make_shared();
  auto test_node = rclcpp::Node::make_shared("test_node");
  auto laser1_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan1", rclcpp::SensorDataQoS().reliable());
  auto laser2_pub = test_node->create_publisher<sensor_msgs::msg::LaserScan>(
    "/scan2", rclcpp::SensorDataQoS().reliable());
  auto laser3d_pub = test_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/pc1", rclcpp::SensorDataQoS().reliable());

  sensor_msgs::msg::PointCloud2::SharedPtr fused_perception;
  auto fused_percept_sub = test_node->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/sensors_node/perceptions", rclcpp::SensorDataQoS().reliable(),
    [&fused_perception](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
      fused_perception = msg;
    });

  auto tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*test_node);
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "odom";
  transform.transform.translation.x = 0.0;
  transform.transform.translation.y = 0.0;
  transform.transform.translation.z = 1.0;

  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(sensors_node->get_node_base_interface());
  exe.add_callback_group(sensors_node->get_real_time_cbg(),
    sensors_node->get_node_base_interface());
  exe.add_node(test_node);

  ASSERT_EQ(std::string(sensors_node->get_name()), "sensors_node");

  std::vector<std::string> sensors = {"laser1", "laser2", "laser3d1"};
  sensors_node->declare_parameter("laser1.topic", std::string("/scan1"));
  sensors_node->declare_parameter("laser1.type", std::string("LaserScan"));
  sensors_node->declare_parameter("laser2.topic", std::string("/scan2"));
  sensors_node->declare_parameter("laser2.type", std::string("LaserScan"));
  sensors_node->declare_parameter("laser3d1.topic", std::string("/pc1"));
  sensors_node->declare_parameter("laser3d1.type", std::string("PointCloud"));

  sensors_node->set_parameter({"sensors", sensors});
  sensors_node->set_parameter({"forget_time", 0.5});
  sensors_node->set_parameter({"laser1.topic", std::string("/scan1")});
  sensors_node->set_parameter({"laser1.type", std::string("LaserScan")});
  sensors_node->set_parameter({"laser2.topic", std::string("/scan2")});
  sensors_node->set_parameter({"laser2.type", std::string("LaserScan")});
  sensors_node->set_parameter({"laser3d1.topic", std::string("/pc1")});
  sensors_node->set_parameter({"laser3d1.type", std::string("PointCloud")});
  {
    auto start = test_node->now();
    while (test_node->now() - start < 100ms) {
      exe.spin_some();
    }
  }

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

  sensors_node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);

  ASSERT_EQ(sensors_node->get_current_state().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_lidar3d";
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      laser2_pub->publish(get_scan_test_4(time2));
      laser3d_pub->publish(get_pc2_test_0(time1));
      exe.spin_some();
    }

    const auto & perceptions = sensors_node->get_perceptions();

    ASSERT_EQ(perceptions.size(), 3u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[1]->stamp).seconds(), 0.0, 0.02);
    ASSERT_EQ(perceptions[1]->valid, true);
    ASSERT_NE(perceptions[1]->subscription, nullptr);
    ASSERT_LT(perceptions[1]->stamp, perceptions[0]->stamp);
    ASSERT_EQ(perceptions[2]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
    ASSERT_EQ(perceptions[2]->valid, true);
    ASSERT_NE(perceptions[2]->subscription, nullptr);
    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 48u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }

  {
    auto start = test_node->now();
    while (test_node->now() - start < 1s) {
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_1";
      tf_broadcaster->sendTransform(transform);
      transform.header.stamp = test_node->now();
      transform.child_frame_id = "base_laser_2";
      tf_broadcaster->sendTransform(transform);
      transform.child_frame_id = "base_lidar3d";
      tf_broadcaster->sendTransform(transform);

      auto time1 = test_node->now();
      auto time2 = time1 - 10ms;

      laser1_pub->publish(get_scan_test_3(time1));
      exe.spin_some();
    }

    const auto & perceptions = sensors_node->get_perceptions();

    ASSERT_EQ(perceptions.size(), 3u);
    ASSERT_EQ(perceptions[0]->data.size(), 16u);
    ASSERT_NEAR((test_node->now() - perceptions[0]->stamp).seconds(), 0.0, 0.001);
    ASSERT_EQ(perceptions[0]->valid, true);
    ASSERT_NE(perceptions[0]->subscription, nullptr);
    ASSERT_EQ(perceptions[1]->data.size(), 16u);
    ASSERT_EQ(perceptions[1]->valid, false);
    ASSERT_NE(perceptions[1]->subscription, nullptr);
    ASSERT_EQ(perceptions[2]->data.size(), 16u);
    ASSERT_EQ(perceptions[2]->valid, false);
    ASSERT_NE(perceptions[2]->subscription, nullptr);

    ASSERT_NE(fused_perception, nullptr);

    pcl::PointCloud<pcl::PointXYZ> fused_pcl;
    pcl::fromROSMsg(*fused_perception, fused_pcl);
    ASSERT_EQ(fused_pcl.points.size(), 16u);

    for (const auto & p : fused_pcl.points) {
      ASSERT_EQ(p.z, 1.0);
    }
  }
  rclcpp::shutdown();
}*/
