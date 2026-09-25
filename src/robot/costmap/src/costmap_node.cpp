#include <chrono>
#include <memory>
#include <cmath>
#include <stdexcept>

#include "costmap_node.hpp"

CostmapNode::CostmapNode() : Node("costmap"), costmap_(this->get_logger()) {
  resolution_ = declare_parameter<double>("resolution", 0.1);
  grid_size_ = declare_parameter<int>("grid_size", 200);
  inflation_radius_ = declare_parameter<double>("inflation_radius", 0.5);
  if (!std::isfinite(resolution_) || resolution_ <= 0.0 ||
      grid_size_ < 2 || grid_size_ > 1000 ||
      !std::isfinite(inflation_radius_) || inflation_radius_ < 0.0 ||
      inflation_radius_ > resolution_ * grid_size_ / 2.0) {
    throw std::invalid_argument("Invalid costmap resolution, grid size, or inflation radius");
  }

  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  // SensorDataQoS accepts the simulator's best-effort lidar messages.
  lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar", rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
      costmap_pub_->publish(costmap_.build(
        *scan, resolution_, grid_size_, inflation_radius_));
    });
  RCLCPP_INFO(get_logger(), "Converting /lidar scans into /costmap (%d x %d cells)",
              grid_size_, grid_size_);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}
