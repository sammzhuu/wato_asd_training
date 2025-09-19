#include "map_memory_node.hpp"

MapMemoryNode::MapMemoryNode() : Node("map_memory"), last_x(0.0), last_y(0.0), distance_threshold(1.5), map_memory_(robot::MapMemoryCore(this->get_logger())) {
    costmap_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/costmap", 10, std::bind(&MapMemoryNode::costmapCallback, this, std::placeholders::_1));
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom/filtered", 10, std::bind(&MapMemoryNode::odomCallback, this, std::placeholders::_1));
      
    // Initialize global map
    initializeGlobal();
    // Initialize publisher
    map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", 10);    
    // Initialize timer
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1), std::bind(&MapMemoryNode::updateMap, this));
}

void MapMemoryNode::initializeGlobal(){
    global_map_.info.resolution = resolution_; // Example resolution
    global_map_.info.width = width_; // Example width
    global_map_.info.height = height_; // Example height
    global_map_.info.origin.position.x =  - (width_ / 2) * resolution_;
    global_map_.info.origin.position.y = - (height_ / 2) * resolution_;
    global_map_.header.frame_id = "sim_world";
    global_map_.data.resize(global_map_.info.width * global_map_.info.height, -1); // Unknown
}

void MapMemoryNode::costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    // Store the latest costmap
    latest_costmap_ = (*msg);
    costmap_updated_ = true;
}

void MapMemoryNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    double new_x = msg->pose.pose.position.x;
    double new_y = msg->pose.pose.position.y;

    // Compute distance traveled
    double distance = std::sqrt(std::pow(new_x - last_x, 2) + std::pow(new_y - last_y, 2));
    if (distance >= distance_threshold) {
        last_x = new_x;
        last_y = new_y;
        should_update_map_ = true;
    }
}
 
// Timer-based map update
void MapMemoryNode::updateMap() {
    if (should_update_map_ && costmap_updated_) {
        integrateCostmap();
        map_pub_->publish(global_map_);
        should_update_map_ = false;
        costmap_updated_ = false;
    }
}

void MapMemoryNode::integrateCostmap() {
    if (latest_costmap_.data.empty()) {
        RCLCPP_WARN(this->get_logger(), "No latest costmap to integrate.");
        return;
    }

    int local_w = latest_costmap_.info.width;
    int local_h = latest_costmap_.info.height;
    double local_res = latest_costmap_.info.resolution;

    int global_w = global_map_.info.width;
    int global_h = global_map_.info.height;
    double global_res = global_map_.info.resolution;

    for (int y = 0; y < local_h; y++) {
        for (int x = 0; x < local_w; x++) {
            int idx_local = y * local_w + x;
            int8_t value = latest_costmap_.data[idx_local];

            if (value == -1) continue; // unknown, skip

            // convert to world coords
            double wx = latest_costmap_.info.origin.position.x + x * local_res;
            double wy = latest_costmap_.info.origin.position.y + y * local_res;

            // convert to global map indices
            int gx = static_cast<int>((wx - global_map_.info.origin.position.x) / global_res);
            int gy = static_cast<int>((wy - global_map_.info.origin.position.y) / global_res);

            if (gx >= 0 && gx < global_w && gy >= 0 && gy < global_h) {
                int idx_global = gy * global_w + gx;
                global_map_.data[idx_global] = value; // overwrite with new data
            }
        }
    }

    // stamp and frame for publishing
    global_map_.header.stamp = this->get_clock()->now();
    global_map_.header.frame_id = "map";
}


// Flags
nav_msgs::msg::OccupancyGrid latest_costmap_;
bool should_update_map_ = false;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MapMemoryNode>());
  rclcpp::shutdown();
  return 0;
}
