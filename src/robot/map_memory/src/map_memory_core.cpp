#include "map_memory_core.hpp"
#include <cmath>

namespace robot {

MapMemoryCore::MapMemoryCore(const rclcpp::Logger& logger,
                             double resolution,
                             int width,
                             int height)
: logger_(logger), resolution_(resolution), width_(width), height_(height) {
    initializeGlobalMap();
}

void MapMemoryCore::initializeGlobalMap() {
    global_map_.info.resolution = resolution_;
    global_map_.info.width = width_;
    global_map_.info.height = height_;
    global_map_.info.origin.position.x = - (width_ / 2.0) * resolution_;
    global_map_.info.origin.position.y = - (height_ / 2.0) * resolution_;
    global_map_.header.frame_id = "sim_world";
    global_map_.data.resize(width_ * height_, -1);
}

void MapMemoryCore::integrateCostmap(const nav_msgs::msg::OccupancyGrid& local_costmap) {
    if (local_costmap.data.empty()) return;

    int local_w = local_costmap.info.width;
    int local_h = local_costmap.info.height;
    double local_res = local_costmap.info.resolution;

    int global_w = global_map_.info.width;
    int global_h = global_map_.info.height;
    double global_res = global_map_.info.resolution;

    double origin_x = local_costmap.info.origin.position.x;
    double origin_y = local_costmap.info.origin.position.y;

    for (int y = 0; y < local_h; ++y) {
        for (int x = 0; x < local_w; ++x) {
            int idx_local = y * local_w + x;
            int8_t value = local_costmap.data[idx_local];

            if (value == -1) continue;

            double wx = origin_x + x * local_res;
            double wy = origin_y + y * local_res;

            int gx = static_cast<int>((wx - global_map_.info.origin.position.x) / global_res);
            int gy = static_cast<int>((wy - global_map_.info.origin.position.y) / global_res);

            if (gx >= 0 && gx < global_w && gy >= 0 && gy < global_h) {
                int idx_global = gy * global_w + gx;
                global_map_.data[idx_global] = value;
            }
        }
    }

    global_map_.header.stamp = rclcpp::Clock().now();
}

const nav_msgs::msg::OccupancyGrid& MapMemoryCore::getGlobalMap() const {
    return global_map_;
}

}  // namespace robot
