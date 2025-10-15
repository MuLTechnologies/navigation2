// Copyright (c) 2019 Intel Corporation
// Copyright (c) 2020 Sarthak Mittal
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

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"

#include "nav2_behavior_tree/plugins/condition/is_pose_away_condition.hpp"

namespace nav2_behavior_tree
{

IsPoseAwayCondition::IsPoseAwayCondition(
  const std::string& condition_name, const BT::NodeConfiguration& conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
}

BT::NodeStatus IsPoseAwayCondition::tick()
{
  geometry_msgs::msg::PoseStamped away_pose;
  double dist_threshold, dist_to_pose;
  getInput("pose", away_pose);
  getInput("distance", dist_threshold);

  if (!nav2_util::getDistanceToPose(dist_to_pose, away_pose,
        *tf_, robot_base_frame_, transform_tolerance_)) {
    return BT::NodeStatus::FAILURE;
  }

  if (dist_to_pose > dist_threshold) {
    return BT::NodeStatus::SUCCESS;
  } else {
    RCLCPP_WARN_STREAM(
      node_->get_logger(), "Provided pose is closer than " << dist_threshold << "m threshold!");
    return BT::NodeStatus::FAILURE;
  }
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::IsPoseAwayCondition>("IsPoseAway");
}
