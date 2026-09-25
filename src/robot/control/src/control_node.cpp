#include "control_node.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>

ControlNode::ControlNode(): Node("control"), control_(get_logger()) {
  options_.lookahead = declare_parameter<double>("lookahead", 0.8);
  options_.max_speed = declare_parameter<double>("max_speed", 0.25);
  options_.max_turn_rate = declare_parameter<double>("max_turn_rate", 0.5);
  options_.goal_tolerance = declare_parameter<double>("goal_tolerance", 0.25);
  data_timeout_ = declare_parameter<double>("data_timeout", 3.0);
  for (double value : {options_.lookahead, options_.max_speed, options_.max_turn_rate,
                        options_.goal_tolerance, data_timeout_})
    if (!std::isfinite(value) || value <= 0.0)
      throw std::invalid_argument("Controller parameters must be finite and positive");
  command_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  path_sub_ = create_subscription<nav_msgs::msg::Path>(
    "/path", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::Path::ConstSharedPtr path) {
      path_ = path;
      path_received_ = std::chrono::steady_clock::now();
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
      // Repeated frozen timestamps do not count as fresh simulation data.
      if (!odom_ || odom->header.stamp != odom_->header.stamp)
        odom_received_ = std::chrono::steady_clock::now();
      odom_ = odom;
    });
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
      if (!scan_ || scan->header.stamp != scan_->header.stamp)
        scan_received_ = std::chrono::steady_clock::now();
      scan_ = scan;
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() { updateControl(); });
  RCLCPP_INFO(get_logger(), "Controller ready: follows /path at up to %.2f m/s",
              options_.max_speed);
}

void ControlNode::updateControl() {
  geometry_msgs::msg::Twist command;
  std::string state = "Stopped: waiting for a planned route";
  const auto now = std::chrono::steady_clock::now();
  const auto fresh = [&](const std::chrono::steady_clock::time_point& received) {
    return std::chrono::duration<double>(now-received).count() <= data_timeout_;
  };
  if (path_ && !path_->poses.empty()) {
    if (!odom_ || !scan_ || !fresh(path_received_) || !fresh(odom_received_) ||
        !fresh(scan_received_)) {
      state = "Stopped: waiting for fresh path, position, and lidar";
    } else if (path_->header.frame_id != odom_->header.frame_id ||
               scan_->header.frame_id != odom_->child_frame_id) {
      state = "Stopped: input coordinate frames do not match";
    } else {
      command = control_.command(*path_, *odom_, options_);
      state = (command.linear.x == 0.0 && command.angular.z == 0.0) ?
        "Stopped: at route endpoint or route is invalid" : "Following planned route";
      // A close obstacle can appear between map updates. Stop forward motion
      // within a rectangle ahead of the lidar; allow turning toward a new route.
      bool valid_beam = false, front_blocked = false, turn_blocked = false;
      const bool valid_scan = std::isfinite(scan_->range_min) &&
        std::isfinite(scan_->range_max) && scan_->range_min >= 0.0 &&
        scan_->range_max > scan_->range_min && std::isfinite(scan_->angle_min) &&
        std::isfinite(scan_->angle_increment);
      if (valid_scan) {
        for (std::size_t i = 0; i < scan_->ranges.size(); ++i) {
          const double range = scan_->ranges[i];
          if (std::isnan(range) || range < scan_->range_min) continue;
          valid_beam = true;
          if (!std::isfinite(range) || range >= scan_->range_max) continue;
          const double angle = scan_->angle_min + i*static_cast<double>(scan_->angle_increment);
          const double x = range*std::cos(angle), y = range*std::sin(angle);
          if (x > 0.0 && x < 0.9 && std::abs(y) < 0.75) front_blocked = true;
          if (range < 0.65) turn_blocked = true;
        }
      }
      if (!valid_beam || (front_blocked && command.linear.x > 0.0) ||
          (turn_blocked && std::abs(command.angular.z) > 0.0)) {
        command = geometry_msgs::msg::Twist();
        state = "Stopped: nearby obstacle or invalid lidar readings";
      }
    }
  }
  command_pub_->publish(command);
  if (state != state_) {
    RCLCPP_INFO(get_logger(), "%s", state.c_str());
    state_ = state;
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
