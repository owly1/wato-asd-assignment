#include "planner_node.hpp"
#include <cmath>
#include <memory>
#include <stdexcept>

PlannerNode::PlannerNode() : Node("planner"), planner_(get_logger()) {
  options_.robot_radius = declare_parameter<double>("robot_radius", 1.5);
  options_.occupied_threshold = declare_parameter<int>("occupied_threshold", 100);
  options_.allow_unknown = declare_parameter<bool>("allow_unknown", true);
  options_.unknown_penalty = declare_parameter<double>("unknown_penalty", 3.0);
  options_.cost_weight = declare_parameter<double>("cost_weight", 2.0);
  options_.clearance_distance = declare_parameter<double>("clearance_distance", 0.8);
  options_.clearance_weight = declare_parameter<double>("clearance_weight", 3.0);
  goal_tolerance_ = declare_parameter<double>("goal_tolerance", 0.3);
  const int period = declare_parameter<int>("replan_period_ms", 1000);
  if (!std::isfinite(options_.robot_radius) || options_.robot_radius < 0.0 ||
      options_.robot_radius > 10.0 || options_.occupied_threshold < 1 ||
      options_.occupied_threshold > 100 || !std::isfinite(options_.unknown_penalty) ||
      options_.unknown_penalty < 0.0 || !std::isfinite(options_.cost_weight) ||
      options_.cost_weight < 0.0 || !std::isfinite(goal_tolerance_) ||
      !std::isfinite(options_.clearance_distance) || options_.clearance_distance < 0.0 ||
      options_.clearance_distance > 5.0 || !std::isfinite(options_.clearance_weight) ||
      options_.clearance_weight < 0.0 || goal_tolerance_ <= 0.0 || period < 100 || period > 5000)
    throw std::invalid_argument("Invalid planner parameters");
  path_pub_ = create_publisher<nav_msgs::msg::Path>(
    "/path", rclcpp::QoS(1).reliable().transient_local());
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).reliable().transient_local(),
    [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
      map_ = map;
      last_map_received_ = std::chrono::steady_clock::now();
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
      odom_ = odom;
      last_odom_received_ = std::chrono::steady_clock::now();
    });
  goal_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
    "/goal_point", 10, [this](geometry_msgs::msg::PointStamped::ConstSharedPtr goal) {
      if (goal->header.frame_id.empty() || !std::isfinite(goal->point.x) ||
          !std::isfinite(goal->point.y) ||
          (map_ && goal->header.frame_id != map_->header.frame_id)) {
        RCLCPP_WARN(get_logger(), "Goal must use the map frame (sim_world) and finite coordinates");
        return;
      }
      goal_ = *goal;
      goal_active_ = true;
      report_next_path_ = true;
      RCLCPP_INFO(get_logger(), "Goal received: (%.2f, %.2f) in %s",
        goal_.point.x, goal_.point.y, goal_.header.frame_id.c_str());
      updatePath();
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(period), [this]() { updatePath(); });
  RCLCPP_INFO(get_logger(), "Planner ready: waiting for a /goal_point in sim_world");
}

void PlannerNode::updatePath() {
  if (!goal_active_) return;
  nav_msgs::msg::Path path;
  path.header.frame_id = map_ ? map_->header.frame_id : goal_.header.frame_id;
  const auto now = std::chrono::steady_clock::now();
  if (!map_ || !odom_ || now - last_map_received_ > std::chrono::seconds(3) ||
      now - last_odom_received_ > std::chrono::seconds(3) ||
      map_->header.frame_id != goal_.header.frame_id ||
      map_->header.frame_id != odom_->header.frame_id) {
    path_pub_->publish(path);
    had_path_ = false;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "Waiting for a current map and position in the goal's coordinate frame");
    return;
  }
  path.header.stamp = odom_->header.stamp;
  if (std::hypot(odom_->pose.pose.position.x - goal_.point.x,
                 odom_->pose.pose.position.y - goal_.point.y) <= goal_tolerance_) {
    goal_active_ = false;
    had_path_ = false;
    path_pub_->publish(path);
    RCLCPP_INFO(get_logger(), "Goal reached; published an empty path to stop control");
    return;
  }
  path = planner_.plan(*map_, *odom_, goal_, options_);
  const bool success = !path.poses.empty();
  if (success && (report_next_path_ || !had_path_)) {
    RCLCPP_INFO(get_logger(), "Planned path with %zu waypoints on /path", path.poses.size());
    report_next_path_ = false;
  } else if (!success) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "No route: goal/start may be outside the map, too near an obstacle, or disconnected");
  }
  had_path_ = success;
  path_pub_->publish(path);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlannerNode>());
  rclcpp::shutdown();
  return 0;
}
