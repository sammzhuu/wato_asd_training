#ifndef PLANNER_CORE_HPP_
#define PLANNER_CORE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>

namespace robot
{

class PlannerCore
{
public:
  explicit PlannerCore(const rclcpp::Logger & logger);

  nav_msgs::msg::Path plan(
    const nav_msgs::msg::OccupancyGrid & map,
    const geometry_msgs::msg::Pose & start,
    const geometry_msgs::msg::Point & goal,
    double inflation_radius_m = 0.5,
    double inflation_weight = 10.0  // NEW: penalty scaling
  );

private:
  rclcpp::Logger logger_;

  inline int idx(int x, int y, int width) const { return y * width + x; }
  inline double heuristic_octile(int x1, int y1, int x2, int y2) const;
  inline bool inBounds(int x, int y, int width, int height) const;

  // now returns a distance transform map instead of binary
  void buildDistanceField(const nav_msgs::msg::OccupancyGrid &map,
                          std::vector<int> &dist_field,
                          int inflation_cells) const;

  bool lineOfSight(const std::vector<int> &dist_field, int x0, int y0, int x1, int y1,
                   int width, int height, const nav_msgs::msg::OccupancyGrid &map) const;
};

} // namespace robot

#endif  // PLANNER_CORE_HPP_
