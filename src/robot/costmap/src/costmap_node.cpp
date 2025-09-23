#include "costmap_node.hpp"
#include <cmath>

CostmapNode::CostmapNode()
: Node("costmap"),
  costmap_core_(this->get_logger(), 0.1, 300, 300),
  robot_x_(0.0),
  robot_y_(0.0),
  robot_yaw_(0.0)
{
    lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/lidar", 10, std::bind(&CostmapNode::laserCallback, this, std::placeholders::_1));

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot/pose", 10, std::bind(&CostmapNode::poseCallback, this, std::placeholders::_1));

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/costmap", 10);
}

void CostmapNode::poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr pose) {
    robot_x_ = pose->pose.position.x;
    robot_y_ = pose->pose.position.y;

    // Convert quaternion to yaw
    double siny_cosp = 2.0 * (pose->pose.orientation.w * pose->pose.orientation.z +
                               pose->pose.orientation.x * pose->pose.orientation.y);
    double cosy_cosp = 1.0 - 2.0 * (pose->pose.orientation.y * pose->pose.orientation.y +
                                    pose->pose.orientation.z * pose->pose.orientation.z);
    robot_yaw_ = std::atan2(siny_cosp, cosy_cosp);
}

void CostmapNode::laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    costmap_core_.initializeCostmap();  // clear old points

    for (size_t i = 0; i < scan->ranges.size(); ++i) {
        double angle = scan->angle_min + i * scan->angle_increment;
        double range = scan->ranges[i];
        costmap_core_.updateFromLaser(range, angle, scan->range_min, scan->range_max,
                                      robot_x_, robot_y_, robot_yaw_);
    }

    costmap_core_.inflateObstacles();
    publishCostmap();
}

void CostmapNode::publishCostmap() {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = this->now();
    grid_msg.header.frame_id = "robot/chassis/lidar";  // global frame

    grid_msg.info.resolution = costmap_core_.getResolution();
    grid_msg.info.width = costmap_core_.getWidth();
    grid_msg.info.height = costmap_core_.getHeight();

    // Center the grid on the robot
    grid_msg.info.origin.position.x = robot_x_ - (grid_msg.info.width / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.position.y = robot_y_ - (grid_msg.info.height / 2.0) * grid_msg.info.resolution;
    grid_msg.info.origin.orientation.w = 1.0;

    const auto &grid = costmap_core_.getGrid();
    grid_msg.data.resize(grid_msg.info.width * grid_msg.info.height);
    for (size_t y = 0; y < grid_msg.info.height; y++) {
        for (size_t x = 0; x < grid_msg.info.width; x++) {
            grid_msg.data[y * grid_msg.info.width + x] = grid[y][x];
        }
    }

    costmap_pub_->publish(grid_msg);
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CostmapNode>());
    rclcpp::shutdown();
    return 0;
}
