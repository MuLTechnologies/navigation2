// Copyright (c) 2019 Intel Corporation
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
#include "nav2_behavior_tree/plugins/control/recovery_node_with_reset.hpp"

namespace nav2_behavior_tree
{

RecoveryNodeWithReset::RecoveryNodeWithReset(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ControlNode::ControlNode(name, conf),
  current_child_idx_(0),
  number_of_retries_(1),
  retry_count_(0)
{
  getInput("number_of_retries", number_of_retries_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);

  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);

  last_retry_pose_ = getRobotPoseInGlobalFrame();
}

BT::NodeStatus RecoveryNodeWithReset::tick()
{
  const unsigned children_count = children_nodes_.size();

  if (children_count != 2) {
    throw BT::BehaviorTreeException(name() + " : Recovery Node must only have 2 children.");
  }

  setStatus(BT::NodeStatus::RUNNING);

  while (current_child_idx_ < children_count && retry_count_ <= number_of_retries_) {
    TreeNode * child_node = children_nodes_[current_child_idx_];
    const BT::NodeStatus child_status = child_node->executeTick();

    if (current_child_idx_ == 0) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // reset node and return success when first child returns success
            halt();
            return BT::NodeStatus::SUCCESS;
          }

        case BT::NodeStatus::FAILURE:
          {
            if (!isRobotCloseToPose(last_retry_pose_)) {
              RCLCPP_INFO_STREAM(node_->get_logger(), name() << " : Retry count restarted because robot is away from last pose.");
              retry_count_ = 0;
            }

            last_retry_pose_ = getRobotPoseInGlobalFrame();
            if (retry_count_ < number_of_retries_) {
              // halt first child and tick second child in next iteration
              ControlNode::haltChild(0);
              current_child_idx_++;
              RCLCPP_INFO_STREAM(node_->get_logger(), name() << " : Retry initiated with retry_count: " << retry_count_);
              break;
            } else {
              // reset node and return failure when max retries has been exceeded
              RCLCPP_INFO_STREAM(node_->get_logger(), name() << " : Retry count exceeded, returning FAILURE");
              halt();
              return BT::NodeStatus::FAILURE;
            }
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError(name() + " : A child node must never return IDLE");
          }
      }  // end switch

    } else if (current_child_idx_ == 1) {
      switch (child_status) {
        case BT::NodeStatus::SUCCESS:
          {
            // halt second child, increment recovery count, and tick first child in next iteration
            ControlNode::haltChild(1);
            retry_count_++;
            current_child_idx_--;
          }
          break;

        case BT::NodeStatus::FAILURE:
          {
            // reset node and return failure if second child fails
            halt();
            RCLCPP_INFO_STREAM(node_->get_logger(), name() << " : Recovery failed, returning FAILURE");

            return BT::NodeStatus::FAILURE;
          }

        case BT::NodeStatus::RUNNING:
          {
            return BT::NodeStatus::RUNNING;
          }

        default:
          {
            throw BT::LogicError(name() + " : A child node must never return IDLE");
          }
      }  // end switch
    }
  }  // end while loop

  // reset node and return failure
  halt();
  return BT::NodeStatus::FAILURE;
}

void RecoveryNodeWithReset::halt()
{
  ControlNode::halt();
  retry_count_ = 0;
  current_child_idx_ = 0;
}

bool RecoveryNodeWithReset::isRobotCloseToPose(geometry_msgs::msg::PoseStamped input_pose)
{
  double distance_to_reset;
  getInput("distance_to_reset", distance_to_reset);

  auto current_pose = getRobotPoseInGlobalFrame();


  double dx = current_pose.pose.position.x - input_pose.pose.position.x;
  double dy = current_pose.pose.position.y - input_pose.pose.position.y;
  double distance = sqrt(dx * dx + dy * dy);
  RCLCPP_INFO_STREAM(node_->get_logger(), name() << " : arePosesNearby dist: " << distance << ", with distance_to_reset: " << distance_to_reset);

  return (dx * dx + dy * dy) <= (distance_to_reset * distance_to_reset);
}

geometry_msgs::msg::PoseStamped RecoveryNodeWithReset::getRobotPoseInGlobalFrame() {
  geometry_msgs::msg::PoseStamped robot_pose;
  if (!nav2_util::getCurrentPose(
      robot_pose, *tf_, global_frame_, robot_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR_STREAM(node_->get_logger(), name() << " : Current robot pose is not available.");
    return geometry_msgs::msg::PoseStamped();
  }
  return robot_pose;
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::RecoveryNodeWithReset>("RecoveryNodeWithReset");
}
