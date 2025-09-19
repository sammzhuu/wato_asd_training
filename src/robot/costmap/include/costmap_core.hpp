#ifndef COSTMAP_CORE_HPP_
#define COSTMAP_CORE_HPP_

#include "rclcpp/rclcpp.hpp"

namespace robot
{

class CostmapCore {
  public:
    // Constructor, we pass in the node's RCLCPP logger to enable logging to terminal
    explicit CostmapCore(const rclcpp::Logger& logger,   double resolution, int width, int height);
    void initializeCostmap();
    void updateFromLaser(double range, double angle, double range_min, double range_max);
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

    void convertToGrid(double range, double angle, int &x_grid, int &y_grid);
    void markObstacle(int x, int y);

};

}  

#endif  