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

#ifndef COGNITIVE_ARCH__BEHAVIOR_TREE_NODES__SEARCH_HPP_
#define COGNITIVE_ARCH__BEHAVIOR_TREE_NODES__SEARCH_HPP_

#include <string>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"

namespace plansys2_search
{

class Search : public BT::ActionNodeBase
{
public:
  explicit Search(
    const std::string & xml_tag_name,
    const BT::NodeConfiguration & conf);

  void halt();
  BT::NodeStatus tick();

  static BT::PortsList providedPorts()
  {
    return BT::PortsList({});
  }

private:
  
  int counter_;
  
  geometry_msgs::msg::Twist giro;
  
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr num_pub;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_kinect;
};

}  // namespace plansys2_search

#endif  // COGNITIVE_ARCH__BEHAVIOR_TREE_NODES__SEARCH_HPP_
