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
#ifndef SCAN_FAKE__NODE_PUB_HPP_
#define SCAN_FAKE__NODE_PUB_HPP_
#include <math.h>
#include <random>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"


class NodePub : public rclcpp::Node
{
public:
  NodePub()
  : Node("node_pub")
  {
    message.angle_min = 0;
    message.angle_max = 2 * M_PI;
    message.angle_increment = 2 * M_PI / 100;
    message.range_min = 0.0;
    message.range_max = 10.0;
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>(
      "scan_fake",
      rclcpp::QoS(100).best_effort());
  }

  void doWork()
  {
    std::normal_distribution<double> distribution(4, 1.0);
    for (int i = 0; i < num_lecturas; i++) {
      double number = distribution(generator);
      while ((number < message.range_min) || (number > message.range_max)) {
        number = distribution(generator);
        message.ranges.push_back(number);
      }
    }

    RCLCPP_INFO(this->get_logger(), "Publishing fake laser");

    pub_->publish(message);
    message.ranges.clear();
  }

private:
  std::default_random_engine generator;

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr pub_;
  sensor_msgs::msg::LaserScan message;
  int num_lecturas = 100;
};
#endif  // SCAN_FAKE__NODE_PUB_HPP_
