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


#pragma once

#include <memory>
#include <string>
#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"

namespace nav2_behavior_tree
{
/**
 * @brief A BT::SyncActionNode that prints message to ROS2
 * logging console with specified verbosity level.
 */
class PrintLog : public BT::SyncActionNode
{
public:
  /**
   * @brief Construct a new Print Log object
   * 
   * @param name 
   * @param config 
   */
  PrintLog(const std::string& name, const BT::NodeConfiguration& config);

  /**
   * @brief The main override required by a BT action
   * @return BT::NodeStatus Status of tick execution
   */
  BT::NodeStatus tick() override;

  /**
   * @brief Creates list of BT ports
   * @return BT::PortsList Containing node-specific ports
   */
  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("message", "PrintLog: default msg"),
      BT::InputPort<std::string>("level", "INFO")
    };
  }

private:
  rclcpp::Logger logger_;
};

}  // namespace nav2_behavior_tree
