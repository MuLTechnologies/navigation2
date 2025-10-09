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

#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_RESET_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_RESET_HPP_

#include <string>
#include "behaviortree_cpp_v3/control_node.h"

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_util/robot_utils.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief The RecoveryNodeWithReset has only two children and returns SUCCESS if and only if the first child
 * returns SUCCESS.
 *
 * - If the first child returns FAILURE, the second child will be executed.  After that the first
 * child is executed again if the second child returns SUCCESS.
 *
 * - If the first or second child returns RUNNING, this node returns RUNNING.
 *
 * - If the second child returns FAILURE, this control node will stop the loop and returns FAILURE.
 * 
 * - The number of retries is calculated. If a certain number is exceeded the whole node returns failure.
 *   The counter is reset if the robot travels a certain distance from the last retry to allow for full number of retres in each scenario.
 *
 */
class RecoveryNodeWithReset : public BT::ControlNode
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::RecoveryNodeWithReset
   * @param name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  RecoveryNodeWithReset(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief A destructor for nav2_behavior_tree::RecoveryNodeWithReset
   */
  ~RecoveryNodeWithReset() override = default;

  /**
   * @brief A method to check if the robot is within a certain tolerance of a pose.
   * 
   * @param input_pose Input pose to check
   * @return state if the robot is within tolerance or not
   */
  bool isRobotCloseToPose(geometry_msgs::msg::PoseStamped input_pose);

  geometry_msgs::msg::PoseStamped getRobotPoseInGlobalFrame();

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing basic ports along with node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<int>("number_of_retries", 1, "Number of retries"),
      BT::InputPort<std::string>("global_frame", "map", "Global frame"),
      BT::InputPort<std::string>("robot_frame", "base_link", "robot frame"),
      BT::InputPort<double>("distance_to_reset", "Distance when checking the last retry pose")
    };
  }

private:
  unsigned int current_child_idx_;
  unsigned int number_of_retries_;
  unsigned int retry_count_;

  double transform_tolerance_;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::string global_frame_;
  std::string robot_frame_;

  geometry_msgs::msg::PoseStamped last_retry_pose_;

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other (optional) override required by a BT action to reset node state
   */
  void halt() override;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__CONTROL__RECOVERY_NODE_WITH_RESET_HPP_
