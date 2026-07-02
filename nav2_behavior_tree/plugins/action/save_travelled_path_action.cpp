#include <string>
#include <memory>

#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"

#include "nav2_behavior_tree/plugins/action/save_travelled_path_action.hpp"

namespace nav2_behavior_tree
{

SaveTravelledPathAction::SaveTravelledPathAction(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(name, conf),
  transform_tolerance_(0.1),
  max_distance_(0.0),
  resolution_(0.1),
  global_frame_("map"),
  robot_frame_("base_link")
{
  getInput("resolution", resolution_);
  getInput("global_frame", global_frame_);
  getInput("robot_frame", robot_frame_);
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");
  node_->get_parameter("transform_tolerance", transform_tolerance_);
  path_publisher_ = node_->create_publisher<nav_msgs::msg::Path>("travelled_path", 1);
}

inline BT::NodeStatus SaveTravelledPathAction::tick()
{
  // Read the max_distance input on every tick in case it is provided as a blackboard variable
  getInput("max_distance", max_distance_);

  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Load the existing trail from the blackboard. Missing/empty entry means
  // start a fresh trail.
  std::vector<geometry_msgs::msg::PoseStamped> previous_poses;
  nav_msgs::msg::Path existing_path;
  if (getInput("path", existing_path)) {
    previous_poses = existing_path.poses;
  }

  // Return immediately if not enough distance was covered
  if (max_distance_ > 0.0 && !previous_poses.empty() &&
    nav2_util::geometry_utils::euclidean_distance(current_pose, previous_poses.back()) <
    resolution_)
  {
    return BT::NodeStatus::SUCCESS;
  }

  // Reset: max_distance == 0.0 clears the buffer, keeping only the current pose
  if (max_distance_ <= 0.0) {
    previous_poses.clear();
    previous_poses.emplace_back(current_pose);
  } else {
    // If the robot moved backward along the trail, the closest buffered pose
    // is no longer the last one. In that case, drop every pose after the closest one
    // so the trail stays a monotonic history of poses truly behind the robot.
    if (previous_poses.size() >= 2) {
      size_t closest_idx = previous_poses.size() - 1;
      double closest_dist = nav2_util::geometry_utils::euclidean_distance(
        current_pose, previous_poses.back());
      for (size_t i = 0; i + 1 < previous_poses.size(); ++i) {
        const double d = nav2_util::geometry_utils::euclidean_distance(
          current_pose, previous_poses[i]);
        if (d < closest_dist) {
          closest_dist = d;
          closest_idx = i;
        }
      }
      if (closest_idx + 1 < previous_poses.size()) {
        previous_poses.erase(
          previous_poses.begin() + closest_idx + 1, previous_poses.end());
      }
    }

    // Append current pose if it's far enough from the last recorded one
    if (previous_poses.empty() ||
      nav2_util::geometry_utils::euclidean_distance(current_pose, previous_poses.back()) >=
      resolution_)
    {
      previous_poses.emplace_back(current_pose);
    }

    // Prune from the front any pose that ended up farther than max_distance behind
    while (!previous_poses.empty() &&
      nav2_util::geometry_utils::euclidean_distance(current_pose, previous_poses.front()) >
      max_distance_)
    {
      previous_poses.erase(previous_poses.begin());
    }
  }

  // Build output path (front = oldest sample, back = current pose)
  nav_msgs::msg::Path output_path;
  output_path.header.frame_id = global_frame_;
  output_path.header.stamp = node_->now();
  output_path.poses = previous_poses;

  setOutput("path", output_path);
  publishPath(output_path);

  return BT::NodeStatus::SUCCESS;
}

void SaveTravelledPathAction::publishPath(const nav_msgs::msg::Path & path)
{
  if (path_publisher_->get_subscription_count() > 0) {
    path_publisher_->publish(path);
  }
}


}  // namespace nav2_behavior_tree

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::SaveTravelledPathAction>("SaveTravelledPath");
}
