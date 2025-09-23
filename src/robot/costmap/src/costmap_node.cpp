#include "costmap_node.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_core_(this->get_logger(), 0.1, 300, 300)
{
    // TF2 setup
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Subscriber & Publisher
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    costmap_core_.initializeCostmap();

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double range = scan->ranges[i];
        if (range < scan->range_min || range > scan->range_max)
            continue;

        double angle = scan->angle_min + i * scan->angle_increment;

        // LiDAR point in local frame
        geometry_msgs::msg::PointStamped lidar_point;
        lidar_point.header.frame_id = "robot/chassis/lidar";
        lidar_point.point.x = range * cos(angle);
        lidar_point.point.y = range * sin(angle);
        lidar_point.point.z = 0.0;

        // Transform to sim_world
        geometry_msgs::msg::PointStamped world_point;
        try {
            world_point = tf_buffer_->transform<geometry_msgs::msg::PointStamped>(
              lidar_point, "sim_world", tf2::durationFromSec(0.1));
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
            continue;
        }

        costmap_core_.updateFromPoint(world_point.point.x, world_point.point.y);
    }

    costmap_core_.inflateObstacles();
    publishCostmap();
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "sim_world";

    grid_msg.info.resolution = costmap_core_.getResolution();
    grid_msg.info.width = costmap_core_.getWidth();
    grid_msg.info.height = costmap_core_.getHeight();
    grid_msg.info.origin.position.x = -(grid_msg.info.width / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.position.y = -(grid_msg.info.height / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.orientation.w = 1.0;

    const auto &grid = costmap_core_.getGrid();
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
    for (size_t y = 0; y < grid_msg.info.height; ++y) {
        for (size_t x = 0; x < grid_msg.info.width; ++x) {
            grid_msg.data[y * grid_msg.info.width + x] = grid[y][x];
        }
    }

    costmap_pub_->publish(grid_msg);
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
