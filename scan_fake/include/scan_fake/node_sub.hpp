#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;


class NodeSub : public rclcpp::Node
{
public:
  NodeSub(const std::string & name)
  : Node(name)
  {
    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_fake", rclcpp::QoS(100).best_effort(), std::bind(&NodeSub::callback, this, _1));
  }

private:
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
      this->get_logger(), "MAX:%.3f\tMIN:%.3f\tAVG:%.3f", max,min,avg);
  }


private:
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Node::SharedPtr node = nullptr;
};