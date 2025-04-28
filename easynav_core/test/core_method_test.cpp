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

#include "easynav_common/types/NavState.hpp"
#include "easynav_core/MethodBase.hpp"
#include "easynav_core/LocalizerMethodBase.hpp"

class CoreMethodTestCase : public ::testing::Test
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

// Mock class to test the behaviour of MethodBase initialization
class MockMethod : public easynav::MethodBase
{
public:
  MockMethod() = default;
  ~MockMethod() = default;

  void on_initialize() override
  {
    on_initialize_called_ = true;
  }

  bool was_on_initialize_called() const
  {
    return on_initialize_called_;
  }

private:
  bool on_initialize_called_ {false};
};

/** Dummy method class to test behvaiour of derived methods */
class TestLocalizer : public easynav::LocalizerMethodBase
{
public:
  TestLocalizer() = default;
  ~TestLocalizer() = default;

  void on_initialize() override
  {
    odom_.header.frame_id = "base_link";
    odom_.pose.pose.position.x = 5;
  }

  [[nodiscard]] virtual nav_msgs::msg::Odometry get_odom() override
  {
    return odom_;
  }

  virtual void update(const easynav::NavState & nav_state) override
  {
    (void) nav_state;
    odom_.pose.pose.position.x = 10;
  }

private:
  nav_msgs::msg::Odometry odom_ {};
};


TEST(MethodBaseTest, DefaultConstructor)
{
  easynav::MethodBase method;
  EXPECT_EQ(
    method.get_node(),
    nullptr
  ) << "Default constructor should initialize parent_node_ to nullptr.";
}

TEST_F(CoreMethodTestCase, InitializeSetsParentNode)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  easynav::MethodBase method;

  method.initialize(node);

  EXPECT_EQ(method.get_node(), node) << "initialize() should set parent_node_ correctly.";
}

TEST_F(CoreMethodTestCase, OnInitializeCalled)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  MockMethod method;

  method.initialize(node);

  EXPECT_TRUE(method.was_on_initialize_called()) <<
    "on_initialize() should be called during initialization.";
}

TEST_F(CoreMethodTestCase, TestDerivedLocalizer)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  TestLocalizer localizer;

  const auto odom_pre = localizer.get_odom();
  EXPECT_EQ(odom_pre.header.frame_id, "") <<
    "Default cosntructor should not initialize odom state";

  localizer.initialize(node);

  const auto odom_init = localizer.get_odom();
  EXPECT_EQ(odom_init.header.frame_id, "base_link") <<
    "initialize should set odom frame_id";
  EXPECT_EQ(odom_init.pose.pose.position.x, 5.0) <<
    "initialize should set odom x position";

  easynav::NavState state;
  localizer.update(state);
  const auto odom_update = localizer.get_odom();
  EXPECT_EQ(odom_update.pose.pose.position.x, 10.0) <<
    "initialize should set odom x position";
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
