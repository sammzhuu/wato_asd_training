#include <chrono>
#include <memory>

#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
 
#include "costmap_node.hpp"

 
CostmapNode::CostmapNode() : Node("costmap"), costmap_core_(this->get_logger(), 0.05, 100, 100) {

  lidar_sub_ = this->create_subscription<std_msgs::msg::LaserScan>("/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

  costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
  string_pub_ = this->create_publisher<std_msgs::msg::String>("/test_topic", 10);

  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CostmapNode::publishMessage, this));
  
} 

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {  
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        costmap_core_.updateFromLaser(range, angle, scan->range_min, scan->range_max);
    }
  costmap_core_.inflateObstacles();
  publishCostmap();
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "map";

    grid_msg.info.resolution = costmap_core_.getResolution();
    grid_msg.info.width = costmap_core_.getWidth();
    grid_msg.info.height = costmap_core_.getHeight();
    grid_msg.info.origin.position.x = -(grid_msg.info.width / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.position.y = -(grid_msg.info.height / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.orientation.w = 1.0;

    const auto &grid = costmap_core_.getGrid();
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
    for (int y = 0; y < grid_msg.info.height; y++) {
        for (int x = 0; x < grid_msg.info.width; x++) {
            grid_msg.data[y * grid_msg.info.width + x] = grid[y][x];
        }
    }
    costmap_pub_->publish(grid_msg);
}

 
// Define the timer to publish a message every 500ms
void CostmapNode::publishMessage() {
  auto message = std_msgs::msg::String();
  message.data = "Hello, ROS 2!";
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  string_pub_->publish(message);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CostmapNode>());
  rclcpp::shutdown();
  return 0;
}