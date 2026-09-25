#ifndef CONTROL_NODE_HPP_
#define CONTROL_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "control_core.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <chrono>
#include <string>

class ControlNode : public rclcpp::Node {
  public:
    ControlNode();

  private:
    robot::ControlCore control_;
    void updateControl();
    robot::PursuitOptions options_;
    double data_timeout_;
    std::string state_;
    nav_msgs::msg::Path::ConstSharedPtr path_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan_;
    std::chrono::steady_clock::time_point path_received_, odom_received_, scan_received_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr command_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
