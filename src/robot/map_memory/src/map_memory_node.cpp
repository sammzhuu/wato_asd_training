#include "map_memory_node.hpp"
#include <cmath>

MapMemoryNode::MapMemoryNode()
: Node("map_memory"), last_x_(0.0), last_y_(0.0), distance_threshold_(1.5),
  costmap_updated_(false), should_update_map_(false),
  map_memory_(this->get_logger())
{
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));

    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);

    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    latest_costmap_ = *msg;
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double new_x = msg->pose.pose.position.x;
    double new_y = msg->pose.pose.position.y;
    double distance = std::sqrt(std::pow(new_x - last_x_, 2) + std::pow(new_y - last_y_, 2));

    if (distance >= distance_threshold_) {
        last_x_ = new_x;
        last_y_ = new_y;
        should_update_map_ = true;
    }
}

void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        map_memory_.integrateCostmap(latest_costmap_);
        map_pub_->publish(map_memory_.getGlobalMap());
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
