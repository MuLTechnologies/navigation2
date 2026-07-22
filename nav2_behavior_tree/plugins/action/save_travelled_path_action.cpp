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
  // start a fresh trail. We work directly on `path.poses` to avoid an extra
  // copy of the pose vector.
  nav_msgs::msg::Path path;
  getInput("path", path);
  auto & poses = path.poses;

  // Distance to the last recorded pose, shared by the checks below
  // (updated if the trail gets truncated after backward motion)
  double dist_to_last_saved_pose = poses.empty() ? 0.0 :
    nav2_util::geometry_utils::euclidean_distance(current_pose, poses.back());

  // Return immediately if not enough distance was covered
  if (max_distance_ > 0.0 && !poses.empty() && dist_to_last_saved_pose < resolution_) {
    return BT::NodeStatus::SUCCESS;
  }

  // Reset: max_distance <= 0.0 clears the buffer, keeping only the current pose.
  // If reset_if_moved > 0.0, keep the existing trail when the robot is still
  // within that distance of the last recorded pose (e.g. a new goal issued from
  // roughly the same spot), so the accumulated backup path is not thrown away.
  if (max_distance_ <= 0.0) {
    double reset_if_moved = 0.0;
    getInput("reset_if_moved", reset_if_moved);
    if (reset_if_moved > 0.0 && !poses.empty() &&
      dist_to_last_saved_pose <= reset_if_moved)
    {
      return BT::NodeStatus::SUCCESS;
    }
    poses.clear();
    poses.emplace_back(current_pose);
  } else {
    // If the robot moved backward along the trail, the closest buffered pose
    // is no longer the last one. In that case, drop every pose after the closest one
    // so the trail stays a monotonic history of poses truly behind the robot.
    if (poses.size() >= 2) {
      size_t closest_idx = poses.size() - 1;
      double closest_dist = dist_to_last_saved_pose;
      for (size_t i = 0; i + 1 < poses.size(); ++i) {
        const double d = nav2_util::geometry_utils::euclidean_distance(
          current_pose, poses[i]);
        if (d < closest_dist) {
          closest_dist = d;
          closest_idx = i;
        }
      }
      if (closest_idx + 1 < poses.size()) {
        poses.erase(poses.begin() + closest_idx + 1, poses.end());
        dist_to_last_saved_pose = closest_dist;
      }
    }

    // Append current pose if it's far enough from the last recorded one
    if (poses.empty() || dist_to_last_saved_pose >= resolution_) {
      poses.emplace_back(current_pose);
    }

    // Prune from the front any pose that ended up farther than max_distance behind
    auto last_pose_within_dist_idx = poses.begin();
    while (last_pose_within_dist_idx != poses.end() &&
      nav2_util::geometry_utils::euclidean_distance(current_pose, *last_pose_within_dist_idx) >
      max_distance_)
    {
      ++last_pose_within_dist_idx;
    }
    poses.erase(poses.begin(), last_pose_within_dist_idx);
  }

  // Refresh header (front = oldest sample, back = current pose)
  path.header.frame_id = global_frame_;
  path.header.stamp = node_->now();

  setOutput("path", path);
  publishPath(path);

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
