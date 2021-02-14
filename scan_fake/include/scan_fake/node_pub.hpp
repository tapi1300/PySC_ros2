#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <random>

class NodePub : public rclcpp::Node
{
public:
  NodePub()
  : Node("node_pub")
  {
    message.angle_min = 0;
    message.angle_max = 2*M_PI;
    message.angle_increment = 2*M_PI/100;
    message.range_min = 0.0;
    message.range_max = 10.0;
    pub_ = create_publisher<sensor_msgs::msg::LaserScan>("scan_fake", rclcpp::QoS(100).best_effort());
  }

  void doWork()
  {
    std::normal_distribution<double> distribution(4,1.0);
    for(int i = 0; i < num_lecturas; i++) {
      double number = distribution(generator);
      while((number < message.range_min) || (number > message.range_max))
        number = distribution(generator);
      message.ranges.push_back(number);
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