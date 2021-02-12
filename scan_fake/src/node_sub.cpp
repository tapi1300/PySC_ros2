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

double accum=0.0;
int counter=0;
double max=0;
double min=10;

rclcpp::Node::SharedPtr node = nullptr;

void callback(const std_msgs::msg::String::SharedPtr msg)
{
  double num=std::stod(msg->data);
  double avg=0;
  accum+=num;
  counter++;
  if (num>max)
    max=num;
  if(num<min)
    min=num;
  avg+=accum/counter;

  RCLCPP_INFO(
    node->get_logger(), "MAX:%.3f   AVG:%.3f   MIN:%.3f   CURRENT:%.3f", max,avg,min,num);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);


  while (rclcpp::ok()) {
    node = rclcpp::Node::make_shared("node_sub");
    auto subscription = node->create_subscription<std_msgs::msg::String>(
      "scan_fake", rclcpp::QoS(100).best_effort(), callback);
    
    rclcpp::spin(node);
  }
  rclcpp::shutdown();

  return 0;
}