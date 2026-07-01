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
  getInput("max_distance", max_distance_);
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
  // Get current pose
  geometry_msgs::msg::PoseStamped current_pose;
  if (!nav2_util::getCurrentPose(
      current_pose, *tf_, global_frame_, robot_frame_,
      transform_tolerance_))
  {
    RCLCPP_ERROR(node_->get_logger(), "Current robot pose is not available.");
    return BT::NodeStatus::FAILURE;
  }

  // Reset: max_distance == 0.0 clears the buffer, keeping only the current pose
  if (max_distance_ <= 0.0) {
    previous_poses_.clear();
    previous_poses_.emplace_back(current_pose);
  } else {
    // Append current pose if it's far enough from the last recorded one
    if (previous_poses_.empty() ||
      nav2_util::geometry_utils::euclidean_distance(current_pose, previous_poses_.back()) >=
      resolution_)
    {
      previous_poses_.emplace_back(current_pose);
    }

    // Prune from the front any pose that ended up farther than max_distance behind
    while (!previous_poses_.empty() &&
      nav2_util::geometry_utils::euclidean_distance(current_pose, previous_poses_.front()) >
      max_distance_)
    {
      previous_poses_.erase(previous_poses_.begin());
    }
  }

  // Build output path (front = oldest sample, back = current pose)
  nav_msgs::msg::Path output_path;
  output_path.header.frame_id = global_frame_;
  output_path.header.stamp = node_->now();
  output_path.poses = previous_poses_;

  setOutput("output_path", output_path);
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
