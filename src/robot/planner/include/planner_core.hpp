#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "grid_search.hpp"

namespace robot
{

class PlannerCore {
  public:
    explicit PlannerCore(const rclcpp::Logger& logger);
    nav_msgs::msg::Path plan(const nav_msgs::msg::OccupancyGrid& map,
                            const nav_msgs::msg::Odometry& odometry,
                            const geometry_msgs::msg::PointStamped& goal,
                            const GridSearchOptions& options) const;

  private:
    rclcpp::Logger logger_;
};

}

#endif
