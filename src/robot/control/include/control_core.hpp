#ifndef CONTROL_CORE_HPP_
#define CONTROL_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pure_pursuit.hpp"

namespace robot
{

class ControlCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    ControlCore(const rclcpp::Logger& logger);
    geometry_msgs::msg::Twist command(const nav_msgs::msg::Path& path,
      const nav_msgs::msg::Odometry& odometry, const PursuitOptions& options) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
