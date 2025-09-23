#ifndef PLANNER_NODE_HPP_
#define PLANNER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "planner_core.hpp"

class PlannerNode : public rclcpp::Node {
public:
  PlannerNode();

private:
  // --- Callbacks ---
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void goalCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void timerCallback();

  // --- Helpers ---
  bool goalReached();
  void planPath();

  // --- ROS Interfaces ---
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // --- State ---
  enum class State { WAITING_FOR_GOAL, WAITING_FOR_ROBOT_TO_REACH_GOAL };
  State state_;

  nav_msgs::msg::OccupancyGrid current_map_;
  geometry_msgs::msg::PointStamped goal_;
  geometry_msgs::msg::Pose robot_pose_;
  bool goal_received_{false};

  // --- Core Planner ---
  robot::PlannerCore planner_;
};

#endif  // PLANNER_NODE_HPP_
