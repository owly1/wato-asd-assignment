#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <string>

namespace robot
{

class MapMemoryCore {
  public:
    explicit MapMemoryCore(const rclcpp::Logger& logger);
    void initialize(double resolution, int grid_size, const std::string& frame);
    bool merge(const nav_msgs::msg::OccupancyGrid& local,
               const nav_msgs::msg::Odometry& odometry);
    const nav_msgs::msg::OccupancyGrid& map() const { return map_; }

  private:
    rclcpp::Logger logger_;
    nav_msgs::msg::OccupancyGrid map_;
};

}

#endif
