// Copyright 2025 malysh
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

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <tutorial_interfaces/msg/num.hpp>

class TestPublisher : public ::testing::Test
{
protected:
  static void SetUpTestCase()
  {
    rclcpp::init(0, nullptr);
  }

  static void TearDownTestCase()
  {
    rclcpp::shutdown();
  }
};

TEST_F(TestPublisher, PublishMessage)
{
  auto node = std::make_shared<rclcpp::Node>("test_node");
  auto publisher = node->create_publisher<tutorial_interfaces::msg::Num>("topic", 10);

  auto received = std::make_shared<bool>(false);
  auto sub = node->create_subscription<tutorial_interfaces::msg::Num>(
    "topic", 10,
    [received](const tutorial_interfaces::msg::Num & msg)
    {
      *received = true;
      EXPECT_GE(msg.num, 0);  // Simple test: num should by >= 0
    });

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  // Publish a message
  tutorial_interfaces::msg::Num msg;
  msg.num = 42;
  publisher->publish(msg);

  // Spin the executor for a while to catch the message
  auto start = std::chrono::steady_clock::now();
  while (!*received && std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  EXPECT_TRUE(*received);
}
