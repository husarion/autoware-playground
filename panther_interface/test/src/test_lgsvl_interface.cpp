// Copyright 2020 The Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "test_panther_interface.hpp"
#include <memory>

TEST_F(PantherInterfaceTest, GearMappingStateCommand)
{
  bool test_completed = false;
  VSC expected_result;

  // Setup subscribtion
  auto handle_state_cmd = [&expected_result, &test_completed]
      (const panther_interface::VSD::SharedPtr msg) -> void {
      EXPECT_EQ(msg->current_gear, expected_result.gear);
      test_completed = true;
    };
  const auto sub_node = std::make_shared<rclcpp::Node>(
    "test_panther_interface_sub_state_command",
    "/gtest");
  auto sub_ptr = sub_node->create_subscription<panther_interface::VSD>(
    sim_state_cmd_topic, rclcpp::QoS(10), handle_state_cmd);

  // Setup Node execution
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);
  executor.add_node(sub_node);
  wait_for_publisher(sub_ptr);

  auto wait_for_subscription_callback =
    [&test_completed, &expected_result, &executor, this](panther_interface::GEAR_TYPE gear,
      panther_interface::GEAR_TYPE expected_gear) -> void {
      auto max_test_dur = std::chrono::seconds(1);
      auto timed_out = false;
      test_completed = false;
      VSC vsc_msg;
      vsc_msg.gear = gear;
      expected_result.gear = expected_gear;
      auto start_time = std::chrono::system_clock::now();

      while (rclcpp::ok() && !test_completed) {
        auto state_command_status = panther_interface_->send_state_command(vsc_msg);
        EXPECT_TRUE(state_command_status);
        executor.spin_some();
        rclcpp::sleep_for(std::chrono::milliseconds(50));
        if (std::chrono::system_clock::now() - start_time > max_test_dur) {
          timed_out = true;
          break;
        }
      }
      EXPECT_FALSE(timed_out);
      EXPECT_TRUE(test_completed);

      // cleaup
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      executor.spin_some();
    };

  // Tests
  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  wait_for_subscription_callback(GearReport::DRIVE_1, panther_interface::VSD::GEAR_DRIVE);

  wait_for_subscription_callback(GearReport::REVERSE, panther_interface::VSD::GEAR_REVERSE);

  wait_for_subscription_callback(
    static_cast<panther_interface::GEAR_TYPE>(99u),
    panther_interface::VSD::GEAR_DRIVE);
}

TEST_F(PantherInterfaceTest, GearMappingStateReport)
{
  lgsvl_msgs::msg::CanBusData cbd_msg;

  // Setup Node execution
  const auto pub_node = std::make_shared<rclcpp::Node>(
    "test_panther_interface_pub_state_report",
    "/gtest");
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node_);

  // Setup Publisher
  const auto pub_ptr = pub_node->create_publisher<lgsvl_msgs::msg::CanBusData>(
    sim_state_rpt_topic,
    rclcpp::QoS{10});
  wait_for_subscriber(pub_ptr);

  auto publish_gear_and_wait =
    [&cbd_msg, &pub_ptr, &executor](panther_interface::GEAR_TYPE gear) -> void {
      cbd_msg.selected_gear = static_cast<int8_t>(gear);
      pub_ptr->publish(cbd_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      executor.spin_some();
    };

  // Tests
  EXPECT_EQ(panther_interface_->get_gear_report().report, 0);

  using GearReport = autoware_auto_vehicle_msgs::msg::GearReport;
  publish_gear_and_wait(panther_interface::VSD::GEAR_DRIVE);
  EXPECT_EQ(panther_interface_->get_gear_report().report, GearReport::DRIVE_1);

  publish_gear_and_wait(panther_interface::VSD::GEAR_REVERSE);
  EXPECT_EQ(panther_interface_->get_gear_report().report, GearReport::REVERSE);

  publish_gear_and_wait(static_cast<panther_interface::GEAR_TYPE>(99u));
  EXPECT_EQ(panther_interface_->get_gear_report().report, GearReport::NEUTRAL);
}
