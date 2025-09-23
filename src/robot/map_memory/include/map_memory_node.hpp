#ifndef MAP_MEMORY_NODE_HPP_
#define MAP_MEMORY_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "map_memory_core.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include <cmath>
#include <string>

class MapMemoryNode : public rclcpp::Node {
  public:
    MapMemoryNode();

  private:
    robot::MapMemoryCore map_memory_;
    // sub and pub 
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // internal state 
    nav_msgs::msg::OccupancyGrid global_map_;
    nav_msgs::msg::OccupancyGrid latest_costmap_;
    double last_x;
    double last_y;
    
    // flags
    bool should_update_map_ = false;
    bool costmap_updated_ = false;

    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void initializeGlobal();
    void updateMap();
    void integrateCostmap();

    // variables for initialization
    static constexpr int width_ = 300;
    static constexpr int height_ = 300;
    static constexpr double resolution_ = 0.1; // meters per cell
    const double distance_threshold = 1.5; // meters
};

#endif 
