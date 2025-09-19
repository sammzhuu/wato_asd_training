#include "costmap_core.hpp"

namespace robot
{

CostmapCore::CostmapCore(const rclcpp::Logger& logger, double resolution, int width, int height) 
: logger_(logger), resolution_(resolution), width_(width), height_(height) {
    initializeCostmap();
}

void CostmapCore::initializeCostmap() {
  grid_.assign(height_, std::vector<int>(width_, 0));
}

void CostmapCore::convertToGrid(double range, double angle, int &x, int &y) {
  double wx = range * cos(angle);
  double wy = range * sin(angle);
  x = static_cast<int>((wx + (width_ * resolution_) / 2.0) / resolution_);
  y = static_cast<int>((wy + (height_ * resolution_) / 2.0) / resolution_);
}

void CostmapCore::markObstacle(int x, int y) {
  if (x >= 0 && x < width_ && y >= 0 && y < height_) {
    grid_[y][x] = 100;
  }
}

void CostmapCore::updateFromLaser(double range, double angle, double range_min, double range_max) {
  if (range > range_min && range < range_max) {
    int gx, gy;
    convertToGrid(range, angle, gx, gy);
    markObstacle(gx, gy);
  }
}

void CostmapCore::inflateObstacles() {
  auto inflated = grid_;
  int inflation_radius_cells = static_cast<int>(1.0 / resolution_);

  for (unsigned int y = 0; y < height_; y++) {
    for (unsigned int x = 0; x < width_; x++) {
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

const std::vector<std::vector<int>>& CostmapCore::getGrid() const {
  return grid_;
}


}