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

#include "nav2_behavior_tree/plugins/action/publish_string_msg_node.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace nav2_behavior_tree
{

PublishStringMsg::PublishStringMsg(const std::string & name, const BT::NodeConfiguration & config)
: BT::SyncActionNode(name, config),
  node_(config.blackboard->get<rclcpp::Node::SharedPtr>("node")),
  logger_(node_->get_logger())
{
}

BT::NodeStatus PublishStringMsg::tick()
{
  std::string topic;
  if (!getInput("topic", topic) || topic.empty()) {
    RCLCPP_ERROR(logger_, "PublishStringMsg: missing or empty 'topic' input port");
    return BT::NodeStatus::FAILURE;
  }

  if (!publisher_ || topic != current_topic_) {
    publisher_ = node_->create_publisher<std_msgs::msg::String>(topic, rclcpp::QoS(10));
    current_topic_ = topic;
  }

  std_msgs::msg::String msg;
  getInput("message", msg.data);

  publisher_->publish(msg);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace nav2_behavior_tree

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<nav2_behavior_tree::PublishStringMsg>("PublishStringMsg");
}
