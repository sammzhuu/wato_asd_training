#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"
#include <vector>

namespace robot {

class CostmapCore {
public:
    CostmapCore(const rclcpp::Logger &logger, double resolution, int width, int height);

    void initializeCostmap();
    void updateFromPoint(double world_x, double world_y); // world frame coords
    void inflateObstacles();
    const std::vector<std::vector<int>>& getGrid() const;

    double getResolution() const { return resolution_; }
    int getWidth() const { return width_; }
    int getHeight() const { return height_; }

private:
    rclcpp::Logger logger_;
    double resolution_;
    int width_, height_;
    std::vector<std::vector<int>> grid_;

    void markObstacle(int x, int y);
};

} // namespace robot

#endif
