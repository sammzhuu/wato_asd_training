#ifndef COSTMAP_NODE_HPP_
#define COSTMAP_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "costmap_core.hpp"

class CostmapNode : public rclcpp::Node {
public:
    CostmapNode();

private:
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
    void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose);
    void publishCostmap();

    // Publishers / Subscribers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Core costmap logic
    robot::CostmapCore costmap_core_;

    // Robot pose
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
};

#endif
