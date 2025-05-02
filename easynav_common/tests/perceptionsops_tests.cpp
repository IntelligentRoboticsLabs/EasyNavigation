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


#include "gtest/gtest.h"
#include <memory>
#include <vector>
#include <cmath>

#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include "easynav_common/types/Perceptions.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

using easynav::Perception;
using easynav::Perceptions;
using easynav::PerceptionsOps;

class PerceptionsOpsTest : public ::testing::Test
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


/// \brief Clone test (deep copy)
TEST_F(PerceptionsOpsTest, CloneOperation)
{
  Perceptions original;
  auto p1 = std::make_shared<easynav::Perception>();
  p1->data.push_back(pcl::PointXYZ(1.0f, 2.0f, 3.0f));
  p1->frame_id = "sensor";
  p1->stamp = rclcpp::Time(1000);
  p1->valid = true;
  original.push_back(p1);

  auto p2 = std::make_shared<easynav::Perception>();
  p2->data.push_back(pcl::PointXYZ(4.0f, 5.0f, 6.0f));
  p2->frame_id = "sensor2";
  p2->stamp = rclcpp::Time(2000);
  p2->valid = true;
  original.push_back(p2);

  auto cloned = easynav::PerceptionsOps(original).clone();

  ASSERT_EQ(cloned.size(), 2u);
  EXPECT_EQ(cloned[0]->frame_id, "sensor");
  EXPECT_EQ(cloned[1]->frame_id, "sensor2");

  EXPECT_EQ(cloned[0]->data.size(), 1u);
  EXPECT_FLOAT_EQ(cloned[0]->data[0].x, 1.0f);
  EXPECT_FLOAT_EQ(cloned[0]->data[0].y, 2.0f);
  EXPECT_FLOAT_EQ(cloned[0]->data[0].z, 3.0f);

  EXPECT_EQ(cloned[1]->data.size(), 1u);
  EXPECT_FLOAT_EQ(cloned[1]->data[0].x, 4.0f);
  EXPECT_FLOAT_EQ(cloned[1]->data[0].y, 5.0f);
  EXPECT_FLOAT_EQ(cloned[1]->data[0].z, 6.0f);

  // Deep copy check
  original[0]->data[0].x = 999.0f;
  EXPECT_FLOAT_EQ(cloned[0]->data[0].x, 1.0f);  // Should be unaffected
}

TEST_F(PerceptionsOpsTest, FilterTest)
{
  Perceptions p;
  auto entry = std::make_shared<Perception>();
  entry->data.push_back(pcl::PointXYZ(1.0, 5.0, 0.5));
  entry->data.push_back(pcl::PointXYZ(1.0, 15.0, 0.5));  // should be filtered out
  entry->data.push_back(pcl::PointXYZ(1.0, 2.0, 0.1));   // should be filtered out
  entry->valid = true;
  p.push_back(entry);

  PerceptionsOps(p).filter({NAN, 0.0, 0.2}, {NAN, 10.0, NAN});

  ASSERT_EQ(p.size(), 1u);
  EXPECT_EQ(p[0]->data.size(), 1u);
  EXPECT_FLOAT_EQ(p[0]->data[0].y, 5.0);
}

TEST_F(PerceptionsOpsTest, CollapseTest)
{
  Perceptions p;
  auto entry = std::make_shared<Perception>();
  entry->data.push_back(pcl::PointXYZ(1.0, 2.0, 0.9));
  entry->data.push_back(pcl::PointXYZ(1.0, 4.0, 1.3));
  entry->valid = true;
  p.push_back(entry);

  PerceptionsOps(p).collapse({NAN, NAN, 0.5});

  for (const auto & pt : p[0]->data) {
    EXPECT_FLOAT_EQ(pt.x, 1.0);
    EXPECT_FLOAT_EQ(pt.z, 0.5);
  }
}

TEST_F(PerceptionsOpsTest, DownsampleTest)
{
  Perceptions p;
  auto entry = std::make_shared<Perception>();

  for (float x = 0.0; x < 1.0; x += 0.05) {
    for (float y = 0.0; y < 1.0; y += 0.05) {
      entry->data.push_back(pcl::PointXYZ(x, y, 0.0));
    }
  }

  entry->valid = true;
  p.push_back(entry);

  size_t before = p[0]->data.size();
  PerceptionsOps(p).downsample(0.2);
  size_t after = p[0]->data.size();

  EXPECT_LT(after, before);
}

/// brief Fuse test (basic TF transform with identity)
TEST_F(PerceptionsOpsTest, FuseOperation)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_fuse_node");
  tf2_ros::Buffer tf_buffer(node->get_clock());
  tf2_ros::TransformListener tf_listener(tf_buffer);

  // Publish two transform
  geometry_msgs::msg::TransformStamped tf1;
  tf1.header.stamp = node->now();
  tf1.header.frame_id = "odom";
  tf1.child_frame_id = "sensor";
  tf1.transform.translation.x = 1.0;
  tf1.transform.translation.y = 0.0;
  tf1.transform.translation.z = 0.0;
  tf1.transform.rotation.w = 1.0;
  tf1.transform.rotation.x = 0.0;
  tf1.transform.rotation.y = 0.0;
  tf1.transform.rotation.z = 0.0;

  geometry_msgs::msg::TransformStamped tf2;
  tf2.header.stamp = node->now();
  tf2.header.frame_id = "odom";
  tf2.child_frame_id = "sensor2";
  tf2.transform.translation.x = -1.0;
  tf2.transform.translation.y = 0.0;
  tf2.transform.translation.z = 0.0;
  tf2.transform.rotation.w = 1.0;
  tf2.transform.rotation.x = 0.0;
  tf2.transform.rotation.y = 0.0;
  tf2.transform.rotation.z = 0.0;

  tf_buffer.setTransform(tf1, "default_authority", false);
  tf_buffer.setTransform(tf2, "default_authority", false);

  // Create perceptions
  Perceptions perceptions;
  auto p1 = std::make_shared<easynav::Perception>();
  p1->data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  p1->frame_id = "sensor";
  p1->stamp = node->now();
  p1->valid = true;
  perceptions.push_back(p1);
  auto p2 = std::make_shared<easynav::Perception>();
  p2->data.push_back(pcl::PointXYZ(1.0, 2.0, 3.0));
  p2->frame_id = "sensor2";
  p2->stamp = node->now();
  p2->valid = true;
  perceptions.push_back(p2);

  tf_buffer.setTransform(tf1, "default_authority", false);
  tf_buffer.setTransform(tf2, "default_authority", false);

  tf1.header.stamp = node->now();
  tf2.header.stamp = node->now();

  tf_buffer.setTransform(tf1, "default_authority", false);
  tf_buffer.setTransform(tf2, "default_authority", false);

  PerceptionsOps(perceptions).fuse("odom", tf_buffer);

  ASSERT_EQ(perceptions.size(), 1u);
  ASSERT_EQ(perceptions[0]->data.size(), 2u);

  EXPECT_FLOAT_EQ(perceptions[0]->data[0].x, 2.0f);
  EXPECT_FLOAT_EQ(perceptions[0]->data[0].y, 2.0f);
  EXPECT_FLOAT_EQ(perceptions[0]->data[0].z, 3.0f);
  EXPECT_FLOAT_EQ(perceptions[0]->data[1].x, 0.0f);
  EXPECT_FLOAT_EQ(perceptions[0]->data[1].y, 2.0f);
  EXPECT_FLOAT_EQ(perceptions[0]->data[1].z, 3.0f);
}
