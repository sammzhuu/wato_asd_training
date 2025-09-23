#include "costmap_core.hpp"
#include <cmath>
#include <algorithm>

namespace robot {

CostmapCore::CostmapCore(const rclcpp::Logger &logger, double resolution, int width, int height)
: logger_(logger), resolution_(resolution), width_(width), height_(height) {
    initializeCostmap();
}

void CostmapCore::initializeCostmap() {
    grid_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapCore::updateFromLaser(double range, double angle, double range_min, double range_max,
                                  double robot_x, double robot_y, double robot_yaw) {
    if (range >= range_min && range <= range_max) {
        int gx, gy;
        convertToGrid(range, angle, robot_x, robot_y, robot_yaw, gx, gy);
        markObstacle(gx, gy);
    }
}

void CostmapCore::convertToGrid(double range, double angle,
                                double robot_x, double robot_y, double robot_yaw,
                                int &x, int &y) {
    double lx = range * cos(angle);
    double ly = range * sin(angle);

    // Rotate and translate to world coordinates
    double wx = cos(robot_yaw) * lx - sin(robot_yaw) * ly + robot_x;
    double wy = sin(robot_yaw) * lx + cos(robot_yaw) * ly + robot_y;

    // Convert to grid indices
    x = static_cast<int>((wx + (width_ * resolution_) / 2.0) / resolution_);
    y = static_cast<int>((wy + (height_ * resolution_) / 2.0) / resolution_);
}

void CostmapCore::markObstacle(int x, int y) {
    if (x >= 0 && x < width_ && y >= 0 && y < height_) {
        grid_[y][x] = 100;
    }
}

void CostmapCore::inflateObstacles() {
    auto inflated = grid_;
    int inflation_radius_cells = static_cast<int>(1.0 / resolution_);

    for (int y = 0; y < height_; y++) {
        for (int x = 0; x < width_; x++) {
            if (grid_[y][x] == 100) {
                for (int dy = -inflation_radius_cells; dy <= inflation_radius_cells; dy++) {
                    for (int dx = -inflation_radius_cells; dx <= inflation_radius_cells; dx++) {
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < width_ && ny >= 0 && ny < height_) {
                            double dist = std::sqrt(dx*dx + dy*dy) * resolution_;
                            if (dist <= 1.0) {
                                int cost = static_cast<int>(100 * (1 - dist / 1.0));
                                inflated[ny][nx] = std::max(inflated[ny][nx], cost);
                            }
                        }
                    }
                }
            }
        }
    }
    grid_ = inflated;
}

const std::vector<std::vector<int>> &CostmapCore::getGrid() const {
    return grid_;
}

}  // namespace robot
