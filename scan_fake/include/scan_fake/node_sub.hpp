
// Copyright 2020 El Grupo del Flow
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
#ifndef SCAN_FAKE__NODE_SUB_HPP_
#define SCAN_FAKE__NODE_SUB_HPP_

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;


class NodeSub : public rclcpp::Node
{
public:
  explicit NodeSub(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::QoS(100).best_effort(), std::bind(&NodeSub::callback, this, _1));
  }

private:
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    double avg = 0.0, max = -1.0, min = -1.0;
    for (int i = 0; i < static_cast<int>(msg->ranges.size()); i++) {
      avg += msg->ranges[i] / msg->ranges.size();
      if (msg->ranges[i] > max || max == -1.0) {
        max = msg->ranges[i];
      }
      if (msg->ranges[i] < min || min == -1.0) {
        min = msg->ranges[i];
      }
    }
    RCLCPP_INFO(
      this->get_logger(), "MAX:%.3f\tMIN:%.3f\tAVG:%.3f", max, min, avg);
  }


private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Node::SharedPtr node = nullptr;
};
#endif  // SCAN_FAKE__NODE_SUB_HPP_
