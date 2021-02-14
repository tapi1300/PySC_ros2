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
#include "sensor_msgs/msg/laser_scan.hpp"

double accum=0.0;
int counter=0;
double max=0;
double min=10;

rclcpp::Node::SharedPtr node = nullptr;

void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
  double avg = 0.0, max = -1.0, min = -1.0;
  for(int i = 0; i < int(msg->ranges.size()); i++) {
    avg += msg->ranges[i]/msg->ranges.size();
    if(msg->ranges[i] > max || max == -1.0) {
      max = msg->ranges[i];
    }
    if(msg->ranges[i] < min || min == -1.0) {
      min = msg->ranges[i];
    }
  }
  RCLCPP_INFO(
    node->get_logger(), "MAX:%.3f\tMIN:%.3f\tAVG:%.3f", max,min,avg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(1);

  while (rclcpp::ok()) {
    node = rclcpp::Node::make_shared("node_sub");
    auto subscription = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::QoS(100).best_effort(), callback);
    
    rclcpp::spin(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();

  return 0;
}