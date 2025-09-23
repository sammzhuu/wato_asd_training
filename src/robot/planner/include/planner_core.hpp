#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <vector>
#include <queue>
#include <cmath>
#include <limits>

namespace robot
{

class PlannerCore {
public:
  // logger is used for RCLCPP logging inside core
  explicit PlannerCore(const rclcpp::Logger & logger);

  // Main entry point: compute a path given map, start pose, and goal point.
  // inflation_radius_m: how many meters around obstacles to mark as occupied
  nav_msgs::msg::Path plan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Point & goal,
    double inflation_radius_m = 0.2);

private:
  rclcpp::Logger logger_;

  // helpers
  inline int idx(int x, int y, int width) const { return y * width + x; }
  bool inBounds(int x, int y, int width, int height) const;
  bool isCellFree(const std::vector<int8_t> &inflated, int x, int y, int width, int height) const;
  void buildInflatedMap(const nav_msgs::msg::OccupancyGrid &map,
                        std::vector<int8_t> &inflated,
                        int inflation_cells) const;

  // heuristic: octile distance for 8-connected grids
  inline double heuristic_octile(int x1, int y1, int x2, int y2) const;

  // line-of-sight check for smoothing using Bresenham
  bool lineOfSight(const std::vector<int8_t> &inflated, int x0, int y0, int x1, int y1,
                   int width, int height, const nav_msgs::msg::OccupancyGrid &map) const;
};

} // namespace robot

#endif // PLANNER_CORE_HPP_
