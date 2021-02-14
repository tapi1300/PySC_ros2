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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "scan_fake/node_sub.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::Rate loop_rate(1);

  auto node_sub = std::make_shared<NodeSub>("node_sub");

  while (rclcpp::ok()) {
    
    rclcpp::spin(node_sub);
    loop_rate.sleep();
  }
 
  rclcpp::shutdown();

  return 0;
}