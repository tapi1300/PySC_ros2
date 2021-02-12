// Copyright 2020 Intelligent Robotics Lab
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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <random>

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("node_pub");
  auto publisher = node->create_publisher<std_msgs::msg::String>(
    "scan_fake", rclcpp::QoS(100).best_effort());
  
  std_msgs::msg::String message;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(4,1.0);

  rclcpp::Rate loop_rate(1);
  while (rclcpp::ok()) {
    double number = distribution(generator);
    if ((number<0.0) || (number>10.0))
      continue;
    message.data = std::to_string(number);

    RCLCPP_INFO(node->get_logger(), "Publishing [%s]", message.data.c_str());

    publisher->publish(message);

    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}