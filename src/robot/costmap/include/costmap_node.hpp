#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_
 
#include "rclcpp/rclcpp.hpp"
 
#include "costmap_core.hpp"
 
class CostmapNode : public rclcpp::Node {
  public:
    CostmapNode();
    
    // Place callback function here
 
  private:
    // Callbacks
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void publishCostmap();
    void publishMessage();

    // Publishers / Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Core costmap logic
    robot::CostmapCore costmap_core_;
};

#endif 