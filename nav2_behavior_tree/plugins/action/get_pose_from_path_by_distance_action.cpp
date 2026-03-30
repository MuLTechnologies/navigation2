#include <limits>
#include <memory>
#include <string>

#include "behaviortree_cpp_v3/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/create_timer_ros.h"

#include "nav2_behavior_tree/plugins/action/get_pose_from_path_by_distance_action.hpp"

namespace nav2_behavior_tree
{

GetPoseFromPathByDistance::GetPoseFromPathByDistance(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf)
{
  tf_buffer_ =
    config().blackboard->template get<std::shared_ptr<tf2_ros::Buffer>>(
    "tf_buffer");
}

inline BT::NodeStatus GetPoseFromPathByDistance::tick()
{
  setStatus(BT::NodeStatus::RUNNING);

  double distance;
  geometry_msgs::msg::PoseStamped pose;
  double angular_distance_weight;
  double max_robot_pose_search_dist;

  getInput("distance", distance);
  getInput("angular_distance_weight", angular_distance_weight);
  getInput("max_robot_pose_search_dist", max_robot_pose_search_dist);

  bool path_pruning = std::isfinite(max_robot_pose_search_dist);
  nav_msgs::msg::Path new_path;
  getInput("input_path", new_path);
  if (!path_pruning || new_path != path_) {
    path_ = new_path;
    closest_pose_detection_begin_ = path_.poses.begin();
  }

  if (!getRobotPose(path_.header.frame_id, pose)) {
    return BT::NodeStatus::FAILURE;
  }

  if (path_.poses.empty()) {
    // Return robot pose if path is empty
    setOutput("output_pose", pose);
    return BT::NodeStatus::SUCCESS;
  }

  auto closest_pose_detection_end = path_.poses.end();
  if (path_pruning) {
    closest_pose_detection_end = nav2_util::geometry_utils::first_after_integrated_distance(
      closest_pose_detection_begin_, path_.poses.end(), max_robot_pose_search_dist);
  }

  // Find the closest pose on the path to the robot
  auto current_pose = nav2_util::geometry_utils::min_by(
    closest_pose_detection_begin_, closest_pose_detection_end,
    [&pose, angular_distance_weight](const geometry_msgs::msg::PoseStamped & ps) {
      return poseDistance(pose, ps, angular_distance_weight);
    });

  if (path_pruning) {
    closest_pose_detection_begin_ = current_pose;
  }

  // Find the pose at the specified distance forward from current pose
  auto forward_pose_it = nav2_util::geometry_utils::first_after_integrated_distance(
    current_pose, path_.poses.end(), distance);

  // Get the pose (forward_pose_it points one past the desired pose, so we need the one before)
  geometry_msgs::msg::PoseStamped output_pose;
  if (forward_pose_it == path_.poses.begin()) {
    // Edge case: distance is 0 or very small
    output_pose = *forward_pose_it;
  } else if (forward_pose_it == path_.poses.end()) {
    // Edge case: distance extends beyond the path, use the last pose
    output_pose = path_.poses.back();
  } else {
    // Normal case: use the pose just before forward_pose_it
    output_pose = *(forward_pose_it - 1);
  }

  // Ensure frame_id is set
  if (output_pose.header.frame_id.empty()) {
    output_pose.header.frame_id = path_.header.frame_id;
  }

  setOutput("output_pose", output_pose);

  return BT::NodeStatus::SUCCESS;
}

inline bool GetPoseFromPathByDistance::getRobotPose(
  std::string path_frame_id, geometry_msgs::msg::PoseStamped & pose)
{
  if (!getInput("pose", pose)) {
    std::string robot_frame;
    if (!getInput("robot_frame", robot_frame)) {
      RCLCPP_ERROR(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Neither pose nor robot_frame specified for %s", name().c_str());
      return false;
    }
    double transform_tolerance;
    getInput("transform_tolerance", transform_tolerance);
    if (!nav2_util::getCurrentPose(
        pose, *tf_buffer_, path_frame_id, robot_frame, transform_tolerance))
    {
      RCLCPP_WARN(
        config().blackboard->get<rclcpp::Node::SharedPtr>("node")->get_logger(),
        "Failed to lookup current robot pose for %s", name().c_str());
      return false;
    }
  }
  return true;
}

double
GetPoseFromPathByDistance::poseDistance(
  const geometry_msgs::msg::PoseStamped & pose1,
  const geometry_msgs::msg::PoseStamped & pose2,
  const double angular_distance_weight)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  // Taking angular distance into account in addition to spatial distance
  // (to improve picking a correct pose near cusps and loops)
  tf2::Quaternion q1;
  tf2::convert(pose1.pose.orientation, q1);
  tf2::Quaternion q2;
  tf2::convert(pose2.pose.orientation, q2);
  double da = angular_distance_weight * std::abs(q1.angleShortestPath(q2));
  return std::sqrt(dx * dx + dy * dy + da * da);
}

}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::GetPoseFromPathByDistance>(
    "GetPoseFromPathByDistance");
}
