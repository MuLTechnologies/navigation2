#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "nav2_mppi_controller/tools/utils.hpp"

namespace
{

geometry_msgs::msg::PoseStamped pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

nav_msgs::msg::Path path(const std::vector<geometry_msgs::msg::PoseStamped> & poses)
{
  nav_msgs::msg::Path p;
  p.header.frame_id = "map";
  p.poses = poses;
  return p;
}

}  // namespace

TEST(PathInversionTest, StraightPathHasNoInversion)
{
  auto p = path({pose(0, 0, 0), pose(0.1, 0, 0), pose(0.2, 0, 0), pose(0.3, 0, 0)});
  EXPECT_EQ(mppi::utils::findFirstPathInversion(p), 4u);
  EXPECT_EQ(mppi::utils::removePosesAfterFirstInversion(p), 0u);
  EXPECT_EQ(p.poses.size(), 4u);
}

TEST(PathInversionTest, DirectCuspIsFoundAtSameIndexAsBefore)
{
  // Forward to x=0.2, then back: cut keeps poses 0..2
  auto p = path({pose(0, 0, 0), pose(0.1, 0, 0), pose(0.2, 0, 0), pose(0.1, 0, 0), pose(0, 0, 0)});
  EXPECT_EQ(mppi::utils::findFirstPathInversion(p), 3u);
}

TEST(PathInversionTest, CuspBehindInPlaceRotationIsFound)
{
  // Reverse leg along -x (heading 0), rotate in place, then drive forward along +x
  auto p = path({
      pose(0.3, 0, 0), pose(0.2, 0, 0), pose(0.1, 0, 0),   // reverse leg
      pose(0.1, 0, 0.4), pose(0.1, 0, 0.8),                // in-place rotation
      pose(0.2, 0, 0.8), pose(0.3, 0.05, 0.8)});           // forward leg
  EXPECT_EQ(mppi::utils::removePosesAfterFirstInversion(p), 5u);
  ASSERT_EQ(p.poses.size(), 5u);
  // Cropped path ends at the turnaround with the rotated heading
  EXPECT_NEAR(p.poses.back().pose.position.x, 0.1, 1e-9);
  EXPECT_NEAR(tf2::getYaw(p.poses.back().pose.orientation), 0.8, 1e-6);
}

TEST(PathInversionTest, ReverseToForwardAtAnAngleIsFound)
{
  // Reverse toward -63 deg (heading ~117), rotate ~99 deg in place, then drive forward toward
  // -141 deg: travel directions are only ~78 deg apart, but the drive direction flips
  const double h1 = 116.6 * M_PI / 180.0, h2 = -144.2 * M_PI / 180.0;
  const double d1 = -63.4 * M_PI / 180.0, d2 = h2;
  auto p = path({
      pose(0, 0, h1), pose(0.05 * std::cos(d1), 0.05 * std::sin(d1), h1),
      pose(0.1 * std::cos(d1), 0.1 * std::sin(d1), h1),
      pose(0.1 * std::cos(d1), 0.1 * std::sin(d1), h1 + 0.8),
      pose(0.1 * std::cos(d1), 0.1 * std::sin(d1), h2),
      pose(0.1 * std::cos(d1) + 0.05 * std::cos(d2), 0.1 * std::sin(d1) + 0.05 * std::sin(d2), h2)});
  EXPECT_EQ(mppi::utils::findFirstPathInversion(p), 5u);
}

TEST(PathInversionTest, RotationWithoutDirectionChangeIsNotAnInversion)
{
  auto p = path({
      pose(0, 0, 0), pose(0.1, 0, 0), pose(0.1, 0, 0.4), pose(0.1, 0, 0.8),
      pose(0.17, 0.07, 0.8), pose(0.24, 0.14, 0.8)});
  EXPECT_EQ(mppi::utils::findFirstPathInversion(p), 6u);
}

TEST(PathInversionTest, LeadingRotationIsIgnored)
{
  auto p = path({
      pose(0, 0, 0), pose(0, 0, 0.4), pose(0, 0, 0.8), pose(0.07, 0.07, 0.8), pose(0.14, 0.14, 0.8)});
  EXPECT_EQ(mppi::utils::findFirstPathInversion(p), 5u);
}
