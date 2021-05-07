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


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include "cognitive_arch/behavior_tree_nodes/Search.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "behaviortree_cpp_v3/behavior_tree.h"


using std::placeholders::_1;
using std::placeholders::_2;

rclcpp::Node::SharedPtr node = nullptr;
bool not_created;
rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_depth;
cv::Point centroid;
geometry_msgs::msg::Point point;
float cx, cy, cz;
double x3,y3,z3;
rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub;
bool notCreated=true;


namespace plansys2_search
{


void f2dto3d(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pc, const int x, const int y)
{
  int postdata =  x * msg_pc->point_step + y * msg_pc->row_step;

  memcpy(&cx, &msg_pc->data[postdata + msg_pc->fields[0].offset], sizeof(float));
  memcpy(&cy, &msg_pc->data[postdata + msg_pc->fields[1].offset], sizeof(float));
  memcpy(&cz, &msg_pc->data[postdata + msg_pc->fields[2].offset], sizeof(float));


  point.x = cx;
  point.y = cy;
  point.z = cz;
}

void publicar_tf(const sensor_msgs::msg::PointCloud2::SharedPtr msg_pc)
{
  std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA\n\n\n"<< std::endl;

  int center_x = centroid.x;
  int center_y = centroid.y;
  f2dto3d(msg_pc, center_x, center_y);

  x3 = point.z;
  y3 = -point.x;
  z3 = -point.y;

  tf2_msgs::msg::TFMessage persona;
  
  // persona.transforms.frame_id = "base_footprint";
  
  geometry_msgs::msg::Quaternion q;
  
  q.x=0;
  q.y=0;
  q.z=0;
  q.w=1;
  
  geometry_msgs::msg::Vector3 translation;
  translation.x=x3;
  translation.y=y3;
  translation.z=z3;
  persona.transforms[0].transform.translation=translation;
  persona.transforms[0].transform.rotation=q;
  // persona.transforms.child_frame_id = "object";
  tf_pub->publish(persona);

}

void callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  cv_bridge::CvImagePtr cv_ptr;

  try{
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch(cv_bridge::Exception &e){
    return;
  }

  cv::Mat hsv_image;
  cv::Mat low_mask, high_mask;

  cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);

  cv::inRange(hsv_image, cv::Scalar(0, 150, 50), cv::Scalar(10, 255, 255), low_mask);
  cv::inRange(hsv_image, cv::Scalar(175, 150, 50), cv::Scalar(180, 255, 255), high_mask);

  cv::Mat mask = low_mask | high_mask;
  cv::Mat kernel = cv::Mat::ones(6, 6, CV_8UC1);

  cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
  cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

  if(cv::countNonZero(mask) > 0){
    // Ha encontrado un objeto Rojo

    cv::Moments mu = cv::moments(mask);

    if(mu.m00 > 0)
    {
      centroid.x = mu.m10/mu.m00;
      centroid.y = mu.m01/mu.m00;

      circle(mask, centroid, 4, cv::Scalar(0,255,0), 3, cv::LINE_AA);

      if (notCreated) {
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_depth;
        tf_pub = node->create_publisher<tf2_msgs::msg::TFMessage>(
          "/tf", 100);
        sub_depth = node->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/depth_registered/points", rclcpp::SensorDataQoS(), publicar_tf);
        notCreated=false;
      }

    }




  }

  cv::imshow("Mask", mask);


  
  }

Search::Search(
  const std::string & xml_tag_name,
  const BT::NodeConfiguration & conf)
: BT::ActionNodeBase(xml_tag_name, conf), counter_(0)
{
    not_created = true;
    node = rclcpp::Node::make_shared("search_aux");
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
  cv::waitKey(1);
  giro.linear.x = 0.0;
  giro.angular.z = -0.30;
  rclcpp::spin_some(node);
  num_pub->publish(giro);
  
  if (counter_++ < 20) {
    return BT::NodeStatus::RUNNING;
  } else {
    counter_ = 0;
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace plansys2_search

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<plansys2_search::Search>("Search");
}