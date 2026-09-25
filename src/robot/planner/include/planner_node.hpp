#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "planner_core.hpp"
#include <chrono>

class PlannerNode : public rclcpp::Node {
  public:
    PlannerNode();

  private:
    robot::PlannerCore planner_;
    void updatePath();
    robot::GridSearchOptions options_;
    double goal_tolerance_;
    bool goal_active_ = false;
    bool report_next_path_ = false;
    bool had_path_ = false;
    geometry_msgs::msg::PointStamped goal_;
    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    nav_msgs::msg::Odometry::ConstSharedPtr odom_;
    std::chrono::steady_clock::time_point last_odom_received_;
    std::chrono::steady_clock::time_point last_map_received_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif
