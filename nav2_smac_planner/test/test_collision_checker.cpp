// Copyright (c) 2020 Shivang Patel
// Copyright (c) 2020 Samsung Research
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <string>
#include <vector>
#include <memory>

#include "gtest/gtest.h"
#include "nav2_smac_planner/collision_checker.hpp"
#include "nav2_util/lifecycle_node.hpp"

using namespace nav2_costmap_2d;  // NOLINT

TEST(collision_footprint, test_basic)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testA");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  geometry_msgs::msg::Point p1;
  p1.x = -0.5;
  p1.y = 0.0;
  geometry_msgs::msg::Point p2;
  p2.x = 0.0;
  p2.y = 0.5;
  geometry_msgs::msg::Point p3;
  p3.x = 0.5;
  p3.y = 0.0;
  geometry_msgs::msg::Point p4;
  p4.x = 0.0;
  p4.y = -0.5;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);
  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_point_cost)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testB");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / pointcose*/, 0.0);

  collision_checker.inCollision(5.0, 5.0, 0.0, false);
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 0.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_world_to_map)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testC");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 0);

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  nav2_costmap_2d::Footprint footprint;
  collision_checker.setFootprint(footprint, true /*radius / point cost*/, 0.0);

  unsigned int x, y;

  collision_checker.worldToMap(1.0, 1.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  float cost = collision_checker.getCost();

  EXPECT_NEAR(cost, 0.0, 0.001);

  costmap->setCost(50, 50, 200);
  collision_checker.worldToMap(5.0, 5.0, x, y);

  collision_checker.inCollision(x, y, 0.0, false);
  EXPECT_NEAR(collision_checker.getCost(), 200.0, 0.001);
  delete costmap_;
}

TEST(collision_footprint, test_footprint_at_pose_with_movement)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testD");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(100, 100, 0.1, 0, 0, 254);

  for (unsigned int i = 40; i <= 60; ++i) {
    for (unsigned int j = 40; j <= 60; ++j) {
      costmap_->setCost(i, j, 128);
    }
  }

  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  float cost = collision_checker.getCost();
  EXPECT_NEAR(cost, 128.0, 0.001);

  EXPECT_TRUE(collision_checker.inCollision(50, 49, 0.0, false));
  float up_value = collision_checker.getCost();
  EXPECT_NEAR(up_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(50, 52, 0.0, false));
  float down_value = collision_checker.getCost();
  EXPECT_NEAR(down_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(11, 11, 0.0, false));
  float other_value = collision_checker.getCost();
  EXPECT_NEAR(other_value, 254.0, 0.001);  // center cost

  delete costmap_;
}

TEST(collision_footprint, test_point_and_line_cost)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testE");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.10000, 0, 0.0, 128.0);

  costmap_->setCost(62, 50, 254);
  costmap_->setCost(39, 60, 254);

  geometry_msgs::msg::Point p1;
  p1.x = -1.0;
  p1.y = 1.0;
  geometry_msgs::msg::Point p2;
  p2.x = 1.0;
  p2.y = 1.0;
  geometry_msgs::msg::Point p3;
  p3.x = 1.0;
  p3.y = -1.0;
  geometry_msgs::msg::Point p4;
  p4.x = -1.0;
  p4.y = -1.0;

  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // Convert raw costmap into a costmap ros object
  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  auto costmap = costmap_ros->getCostmap();
  *costmap = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  float value = collision_checker.getCost();
  EXPECT_NEAR(value, 128.0, 0.001);

  EXPECT_TRUE(collision_checker.inCollision(49, 50, 0.0, false));
  float left_value = collision_checker.getCost();
  EXPECT_NEAR(left_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(52, 50, 0.0, false));
  float right_value = collision_checker.getCost();
  EXPECT_NEAR(right_value, 128.0, 0.001);  // center cost

  EXPECT_TRUE(collision_checker.inCollision(39, 60, 0.0, false));
  float other_value = collision_checker.getCost();
  EXPECT_NEAR(other_value, 254.0, 0.001);  // center cost

  delete costmap_;
}

TEST(collision_footprint, test_score_footprint)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testF");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.1, 0.0, 0.0, 0);

  // Cart-like rectangle with base_link near the front: x in [-1.0, 0.2], y in [-0.3, 0.3]
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = 0.2;
  p1.y = 0.3;
  p2.x = 0.2;
  p2.y = -0.3;
  p3.x = -1.0;
  p3.y = -0.3;
  p4.x = -1.0;
  p4.y = 0.3;
  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // At (50, 50) heading 0: rear-left corner is cell (40, 53), front midpoint is (52, 50)
  costmap_->setCost(40, 53, 100);
  costmap_->setCost(52, 50, 200);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  // Collision result and getCost() are unchanged: center cell only
  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  EXPECT_NEAR(collision_checker.getCost(), 0.0, 0.001);

  // 4 vertices + 4 edge midpoints + centroid = 9 samples
  EXPECT_NEAR(collision_checker.scoreFootprint(50, 50, 0.0, false), 300.0 / 9.0, 0.001);
  EXPECT_NEAR(collision_checker.scoreFootprint(50, 50, 0.0, true), 200.0, 0.001);
  EXPECT_NEAR(collision_checker.getCost(), 0.0, 0.001);

  // Rotated 90 deg (bin 18 of 72), no sample lands on either costed cell
  EXPECT_NEAR(collision_checker.scoreFootprint(50, 50, 18.0, false), 0.0, 0.001);

  // Samples off the map score as occupied
  EXPECT_NEAR(collision_checker.scoreFootprint(5, 50, 0.0, true), 254.0, 0.001);

  // Radius footprints fall back to the center cell
  collision_checker.setFootprint(footprint, true /*radius*/, 0.0);
  EXPECT_NEAR(collision_checker.scoreFootprint(52, 50, 0.0, false), 200.0, 0.001);

  delete costmap_;
}

TEST(collision_footprint, test_footprint_cost_between_samples)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testG");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.1, 0.0, 0.0, 0);

  // Same cart-like rectangle as test_score_footprint: x in [-1.0, 0.2], y in [-0.3, 0.3]
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = 0.2;
  p1.y = 0.3;
  p2.x = 0.2;
  p2.y = -0.3;
  p3.x = -1.0;
  p3.y = -0.3;
  p4.x = -1.0;
  p4.y = 0.3;
  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // At (50, 50) heading 0 the left side runs along row 53 from x 40 to 52; samples sit at
  // 40, 46 and 52, so cell 43 (a quarter point) is on the outline but between samples
  costmap_->setCost(43, 53, 200);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 72, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_EQ(collision_checker.getFootprintCost(), -1.0f);
  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  // The samples miss the cell; the rasterized outline doesn't
  EXPECT_NEAR(collision_checker.scoreFootprint(50, 50, 0.0, true), 0.0, 0.001);
  EXPECT_NEAR(collision_checker.getFootprintCost(), 200.0, 0.001);

  // Reset on every check: a pose far from the costed cell rasterizes to 0
  EXPECT_FALSE(collision_checker.inCollision(20, 20, 0.0, false));
  EXPECT_NEAR(collision_checker.getFootprintCost(), 0.0, 0.001);

  // Not rasterized when the center is below the possibly-inscribed threshold
  collision_checker.setFootprint(footprint, false, 100.0);
  EXPECT_FALSE(collision_checker.inCollision(50, 50, 0.0, false));
  EXPECT_EQ(collision_checker.getFootprintCost(), -1.0f);

  delete costmap_;
}

TEST(collision_footprint, test_angle_bin_wraps)
{
  auto node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("testG");
  nav2_costmap_2d::Costmap2D * costmap_ = new nav2_costmap_2d::Costmap2D(
    100, 100, 0.1, 0.0, 0.0, 0);

  // Same cart-like rectangle as test_score_footprint: x in [-1.0, 0.2], y in [-0.3, 0.3]
  geometry_msgs::msg::Point p1, p2, p3, p4;
  p1.x = 0.2;
  p1.y = 0.3;
  p2.x = 0.2;
  p2.y = -0.3;
  p3.x = -1.0;
  p3.y = -0.3;
  p4.x = -1.0;
  p4.y = 0.3;
  nav2_costmap_2d::Footprint footprint = {p1, p2, p3, p4};

  // On the left side at heading 0 (row 53, x 40 to 52); off the outline at heading pi
  costmap_->setCost(43, 53, 254);

  auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>();
  costmap_ros->on_configure(rclcpp_lifecycle::State());
  *costmap_ros->getCostmap() = *costmap_;

  nav2_smac_planner::GridCollisionChecker collision_checker(costmap_ros, 16, node);
  collision_checker.setFootprint(footprint, false /*use footprint*/, 0.0);

  EXPECT_TRUE(collision_checker.inCollision(50, 50, 0.0, false));
  EXPECT_FALSE(collision_checker.inCollision(50, 50, 8.0, false));
  // A primitive pose at exactly 2*pi gives bin == number of bins: same as bin 0
  EXPECT_TRUE(collision_checker.inCollision(50, 50, 16.0, false));

  delete costmap_;
}

int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  rclcpp::init(0, nullptr);

  int result = RUN_ALL_TESTS();

  rclcpp::shutdown();

  return result;
}
