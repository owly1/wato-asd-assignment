#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger);

    nav_msgs::msg::OccupancyGrid build(
      const sensor_msgs::msg::LaserScan& scan,
      double resolution, int grid_size, double inflation_radius) const;

  private:
    rclcpp::Logger logger_;

};

}

#endif
