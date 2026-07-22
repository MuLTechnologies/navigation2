#ifndef NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SAVE_TRAVELLED_PATH_ACTION_HPP_
#define NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SAVE_TRAVELLED_PATH_ACTION_HPP_

#include <vector>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "behaviortree_cpp_v3/action_node.h"

namespace nav2_behavior_tree
{

/**
 * @brief A BT::ActionNode that records the robot's trajectory over the last
 * max_distance meters and outputs it as a nav_msgs::msg::Path.
 *
 * A new pose sample is appended each tick, but only if the robot has moved
 * at least resolution meters since the last sample. Poses that end up
 * farther than max_distance behind the current position are pruned.
 *
 * resolution therefore controls both the spacing of consecutive samples
 * and, effectively, the accuracy with which the tail of the path tracks
 * max_distance.
 *
 * Calling this node with max_distance = 0.0 clears the recorded path
 * (only the current pose remains), unless reset_if_moved > 0.0 and the robot
 * is still within that distance of the last recorded pose, in which case the
 * existing trail is reused instead of cleared.
 *
 * Returns BT::NodeStatus::SUCCESS unless the current robot pose cannot be
 * obtained (FAILURE).
 */
class SaveTravelledPathAction : public BT::ActionNodeBase
{
public:
  /**
   * @brief A constructor for nav2_behavior_tree::SaveTravelledPathAction
   * @param xml_tag_name Name for the XML tag for this node
   * @param conf BT node configuration
   */
  SaveTravelledPathAction(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::BidirectionalPort<nav_msgs::msg::Path>(
        "path",
        "The path covered by the robot over the last max_distance meters. "
        "Read on tick to continue an existing trail, then written back. "
        "Set 0.0 to reset."),
      BT::InputPort<double>(
        "max_distance", 0.0,
        "Length of the recorded backup path behind the robot"),
      BT::InputPort<double>(
        "resolution", 0.1,
        "Minimum spacing between consecutive samples on the backup path"),
      BT::InputPort<double>(
        "reset_if_moved", 0.0,
        "In reset mode (max_distance <= 0.0), only clear the trail if the robot "
        "moved more than this many meters from the last recorded pose; otherwise "
        "reuse the existing trail. 0.0 (default) always clears."),
      BT::InputPort<std::string>(
        "global_frame", std::string("map"), "Global frame"),
      BT::InputPort<std::string>(
        "robot_frame", std::string("base_link"), "Robot base frame")
    };
  }

private:
  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief The other override required by a BT action.
   */
  void halt() override {}

  /**
   * @brief Publishes the output path as a nav_msgs::msg::Path message
   *
   * @param path
   */
  void publishPath(const nav_msgs::msg::Path & path);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  double transform_tolerance_;
  double max_distance_, resolution_;
  std::string global_frame_, robot_frame_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
};

}  // namespace nav2_behavior_tree

#endif  // NAV2_BEHAVIOR_TREE__PLUGINS__ACTION__SAVE_TRAVELLED_PATH_ACTION_HPP_
