#include "map_memory_node.hpp"
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

MapMemoryNode::MapMemoryNode() : Node("map_memory"), map_memory_(get_logger()) {
  const double resolution = declare_parameter<double>("resolution", 0.1);
  const int grid_size = declare_parameter<int>("grid_size", 600);
  const std::string frame = declare_parameter<std::string>("map_frame", "sim_world");
  const int update_ms = declare_parameter<int>("update_period_ms", 500);
  max_pose_age_ = declare_parameter<double>("max_pose_age", 0.5);
  if (!std::isfinite(resolution) || resolution <= 0.0 ||
      grid_size < 2 || grid_size > 2000 || frame.empty() || update_ms < 50 ||
      !std::isfinite(max_pose_age_) || max_pose_age_ <= 0.0)
    throw std::invalid_argument("Invalid map memory parameters");
  map_memory_.initialize(resolution, grid_size, frame);
  map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/map", rclcpp::QoS(1).transient_local().reliable());
  costmap_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/costmap", 10, [this](nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) {
      latest_costmap_ = map;
      costmap_pending_ = true;
    });
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
    "/odom/filtered", rclcpp::SensorDataQoS(),
    [this](nav_msgs::msg::Odometry::ConstSharedPtr odom) {
      odometry_history_.push_back(odom);
      if (odometry_history_.size() > 200) odometry_history_.pop_front();
    });
  timer_ = create_wall_timer(std::chrono::milliseconds(update_ms),
                             [this]() { updateMap(); });
  // Publish even while stationary or waiting for a scan, so /map exists at startup.
  map_pub_->publish(map_memory_.map());
  RCLCPP_INFO(get_logger(), "Map memory ready: %d x %d cells in %s",
              grid_size, grid_size, frame.c_str());
}

void MapMemoryNode::updateMap() {
  if (costmap_pending_ && latest_costmap_ && !odometry_history_.empty()) {
    nav_msgs::msg::Odometry::ConstSharedPtr closest;
    double best_age = std::numeric_limits<double>::infinity();
    const auto seconds = [](const builtin_interfaces::msg::Time& stamp) {
      return static_cast<double>(stamp.sec) + stamp.nanosec * 1e-9;
    };
    const double scan_time = seconds(latest_costmap_->header.stamp);
    for (const auto& odom : odometry_history_) {
      const double age = std::abs(seconds(odom->header.stamp) - scan_time);
      if (age < best_age) { best_age = age; closest = odom; }
    }
    // Use the pose nearest the scan's timestamp, rather than a later robot pose.
    if (closest && best_age <= max_pose_age_) {
      if (map_memory_.merge(*latest_costmap_, *closest)) {
        costmap_pending_ = false;
        if (first_merge_) {
          RCLCPP_INFO(get_logger(), "Merged first costmap into /map using /odom/filtered");
          first_merge_ = false;
        }
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Cannot merge costmap: check frame IDs and message contents");
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for odometry close to the costmap timestamp");
    }
  }
  map_pub_->publish(map_memory_.map());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
