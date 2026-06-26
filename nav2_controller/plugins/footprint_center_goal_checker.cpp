/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2017, Locus Robotics
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include "nav2_controller/plugins/footprint_center_goal_checker.hpp"
#include "nav2_util/node_utils.hpp"
#include "pluginlib/class_list_macros.hpp"
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#include "tf2/utils.h"
#pragma GCC diagnostic pop

namespace nav2_controller
{

FootprintCenterGoalChecker::FootprintCenterGoalChecker()
: SimpleGoalChecker(), footprint_x_offset_(0.0), default_x_offset_(-0.5), offset_computed_(false)
{
}

void FootprintCenterGoalChecker::initialize(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  const std::string & plugin_name,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  SimpleGoalChecker::initialize(parent, plugin_name, costmap_ros);
  costmap_ros_ = costmap_ros;
  node_ = parent;

  auto node = parent.lock();
  nav2_util::declare_parameter_if_not_declared(
    node, plugin_name + ".default_x_offset", rclcpp::ParameterValue(-0.5));
  node->get_parameter(plugin_name + ".default_x_offset", default_x_offset_);

  computeFootprintOffset();
}

void FootprintCenterGoalChecker::computeFootprintOffset()
{
  auto node = node_.lock();
  const auto & footprint = costmap_ros_->getUnpaddedRobotFootprint();
  if (footprint.empty()) {
    footprint_x_offset_ = default_x_offset_;
    offset_computed_ = true;
    RCLCPP_WARN(
      node->get_logger(),
      "FootprintCenterGoalChecker: footprint not available, using default_x_offset=%.4f m",
      default_x_offset_);
    return;
  }

  double max_x = std::max_element(
    footprint.begin(), footprint.end(),
    [](const auto & a, const auto & b) {return a.x < b.x;})->x;
  double min_x = std::min_element(
    footprint.begin(), footprint.end(),
    [](const auto & a, const auto & b) {return a.x < b.x;})->x;
  footprint_x_offset_ = (max_x + min_x) / 2.0;
  offset_computed_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "FootprintCenterGoalChecker: base_link->footprint center offset: %.4f m",
    footprint_x_offset_);
}

bool FootprintCenterGoalChecker::isGoalReached(
  const geometry_msgs::msg::Pose & query_pose, const geometry_msgs::msg::Pose & goal_pose,
  const geometry_msgs::msg::Twist & velocity)
{
  if (!offset_computed_) {
    computeFootprintOffset();
  }
  double yaw = tf2::getYaw(query_pose.orientation);
  geometry_msgs::msg::Pose shifted = query_pose;
  shifted.position.x += footprint_x_offset_ * std::cos(yaw);
  shifted.position.y += footprint_x_offset_ * std::sin(yaw);
  return SimpleGoalChecker::isGoalReached(shifted, goal_pose, velocity);
}

}  // namespace nav2_controller

PLUGINLIB_EXPORT_CLASS(nav2_controller::FootprintCenterGoalChecker, nav2_core::GoalChecker)
