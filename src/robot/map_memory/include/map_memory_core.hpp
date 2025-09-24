#ifndef MAP_MEMORY_CORE_HPP_
#define MAP_MEMORY_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace robot {

class MapMemoryCore {
public:
    explicit MapMemoryCore(const rclcpp::Logger& logger,
                           double resolution = 0.1,
                           int width = 300,
                           int height = 300);

    void initializeGlobalMap();
    void integrateCostmap(const nav_msgs::msg::OccupancyGrid& local_costmap);
    const nav_msgs::msg::OccupancyGrid& getGlobalMap() const;

private:
    rclcpp::Logger logger_;
    double resolution_;
    int width_, height_;
    nav_msgs::msg::OccupancyGrid global_map_;
};

}  // namespace robot

#endif
