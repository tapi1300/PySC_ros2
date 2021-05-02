// Copyright 2019 El grupo del Flow
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

#include <string>
#include <iostream>

#include "cognitive_arch/behavior_tree_nodes/Search.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"

using std::placeholders::_1;
using std::placeholders::_2;

rclcpp::Node::SharedPtr node = nullptr;
namespace plansys2_search
{

void callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  std::cout << "TUS MUERTOS" << std::endl;
}

Search::Search(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
    node = rclcpp::Node::make_shared("simple_node_pub");
    num_pub = node->create_publisher<geometry_msgs::msg::Twist>(
    "/cmd_vel", 100);
    sub_kinect = node->create_subscription<sensor_msgs::msg::Image>(
      "/kinect_color/image_raw", rclcpp::SensorDataQoS(), callback);
    
}



void
Search::halt()
{
  std::cout << "Search halt" << std::endl;
}

BT::NodeStatus
Search::tick()
{
  std::cout << "Search tick " << counter_ << std::endl;
  
  giro.linear.x = 0.0;
  giro.angular.z = -0.30;
  rclcpp::spin_some(node);
  num_pub->publish(giro);
//   if (objecto_encontrado) {
//       palablackboard;
//   }
  if (counter_++ < 20) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_search

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_search::Search>("Search");
}