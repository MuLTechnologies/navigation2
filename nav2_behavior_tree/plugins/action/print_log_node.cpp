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

#include "nav2_behavior_tree/plugins/action/print_log_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree {

PrintLog::PrintLog(const std::string &name, const BT::NodeConfiguration &config)
    : BT::SyncActionNode(name, config),
      logger_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")
                  ->get_logger()) {}

BT::NodeStatus PrintLog::tick() {
  std::string message;
  std::string level;

  getInput("message", message);
  getInput("level", level);

  if (level == "DEBUG") {
    RCLCPP_DEBUG_STREAM(logger_, message);
  } else if (level == "INFO") {
    RCLCPP_INFO_STREAM(logger_, message);
  } else if (level == "WARN") {
    RCLCPP_WARN_STREAM(logger_, message);
  } else if (level == "ERROR") {
    RCLCPP_ERROR_STREAM(logger_, message);
  } else {
    RCLCPP_WARN_STREAM(logger_, "Undefined severity: " << message);
  }

  return BT::NodeStatus::SUCCESS;
}

BT_REGISTER_NODES(factory) {
  factory.registerNodeType<nav2_behavior_tree::PrintLog>("PrintLog");
}

} // namespace nav2_behavior_tree
