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
#include <math.h>
#include <random>


using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  printf("wtf");
  rclcpp::init(argc, argv);

  int num_lecturas = 100;

  auto node = rclcpp::Node::make_shared("node_pub");
  auto publisher = node->create_publisher<sensor_msgs::msg::LaserScan>(
    "scan_fake", rclcpp::QoS(100).best_effort());
  
  sensor_msgs::msg::LaserScan message;
  message.angle_min = 0;
  message.angle_max = 2*M_PI;
  message.angle_increment = 2*M_PI/100;
  message.range_min = 0.0;
  message.range_max = 10.0;

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(4,1.0);


  rclcpp::Rate loop_rate(1);

  while (rclcpp::ok()) {
    for(int i = 0; i < num_lecturas; i++) {
      double number = distribution(generator);
      while((number < message.range_min) || (number > message.range_max))
        double number = distribution(generator);
      message.ranges.push_back(number);
    }

    RCLCPP_INFO(node->get_logger(), "Publishing fake laser");

    publisher->publish(message);
    
    message.ranges.clear();


    rclcpp::spin_some(node);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}