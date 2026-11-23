#include <gtest/gtest.h>
#include <chrono>
#include <memory>
#include <thread>

#include "../../test_behavior_tree_fixture.hpp"
#include "nav2_behavior_tree/plugins/decorator/timeout_controller.hpp"

using namespace std::chrono_literals;  // NOLINT

// A child that counts how often it is halted
class HaltCountingNode : public nav2_behavior_tree::DummyNode
{
public:
  void halt() override
  {
    ++halt_count;
  }

  int halt_count{0};
};

class TimeoutControllerTestFixture : public nav2_behavior_tree::BehaviorTreeTestFixture
{
public:
  void SetUp()
  {
    BT::NodeConfiguration conf = *config_;
    conf.input_ports["timeout"] = "0.3";
    bt_node_ = std::make_shared<nav2_behavior_tree::TimeoutController>(
      "timeout_controller", conf);
    child_ = std::make_shared<HaltCountingNode>();
    bt_node_->setChild(child_.get());
  }

  void TearDown()
  {
    child_.reset();
    bt_node_.reset();
  }

protected:
  static std::shared_ptr<nav2_behavior_tree::TimeoutController> bt_node_;
  static std::shared_ptr<HaltCountingNode> child_;
};

std::shared_ptr<nav2_behavior_tree::TimeoutController>
TimeoutControllerTestFixture::bt_node_ = nullptr;
std::shared_ptr<HaltCountingNode>
TimeoutControllerTestFixture::child_ = nullptr;

TEST_F(TimeoutControllerTestFixture, test_child_result_is_passed_on)
{
  child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);

  child_->changeStatus(BT::NodeStatus::SUCCESS);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::SUCCESS);

  child_->changeStatus(BT::NodeStatus::FAILURE);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);

  EXPECT_EQ(child_->halt_count, 0);
}

TEST_F(TimeoutControllerTestFixture, test_timeout_halts_child)
{
  child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child_->halt_count, 0);

  // Timed out: FAILURE, and the child is not left running
  std::this_thread::sleep_for(400ms);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::FAILURE);
  EXPECT_EQ(child_->halt_count, 1);
  EXPECT_EQ(child_->status(), BT::NodeStatus::IDLE);

  // The next tick starts a new time window and ticks the child again
  child_->changeStatus(BT::NodeStatus::RUNNING);
  EXPECT_EQ(bt_node_->executeTick(), BT::NodeStatus::RUNNING);
  EXPECT_EQ(child_->halt_count, 1);
}

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // initialize ROS
  rclcpp::init(argc, argv);

  int all_successful = RUN_ALL_TESTS();

  // shutdown ROS
  rclcpp::shutdown();

  return all_successful;
}
