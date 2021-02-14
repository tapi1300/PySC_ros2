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
#include <math.h>
#include <random>
#include "scan_fake/node_pub.hpp"


using namespace std::chrono_literals;

int main(int argc, char * argv[])
{

  rclcpp::init(argc, argv);

  auto node_pub = std::make_shared<NodePub>();

  rclcpp::Rate loop_rate(1);

  while (rclcpp::ok()) {
    node_pub->doWork();
    rclcpp::spin_some(node_pub);
    loop_rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}