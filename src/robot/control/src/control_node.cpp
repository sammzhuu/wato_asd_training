#include "control_node.hpp"
#include <chrono>

using namespace std::chrono_literals;

ControlNode::ControlNode() : Node("pure_pursuit_controller") {
  // Declare parameters with consistent names
  lookahead_distance_ = this->declare_parameter("lookahead_distance", 1.0);
  goal_tolerance_     = this->declare_parameter("goal_tolerance", 0.05);
  linear_speed_       = this->declare_parameter("linear_speed", 0.5);
  linear_kp_          = this->declare_parameter("linear_kp", 0.5);
  angular_kp_         = this->declare_parameter("angular_kp", 1.0);
  min_linear_speed_   = this->declare_parameter("min_linear_speed", 0.05);   // ensure we still inch forward
  max_angular_speed_  = this->declare_parameter("max_angular_speed", 1.0);    // rad/s clamp
  slowdown_radius_    = this->declare_parameter("slowdown_radius", 0.5);     // start slowing before final

  // Subscribers
  path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
      "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

  // Publisher
  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

  // Timer: 10 Hz control loop
  control_timer_ = this->create_wall_timer(100ms, [this]() { controlLoop(); });

  RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized.");
}

void ControlNode::controlLoop() {
  if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
    return;
  }

  // Check goal condition
  double goal_dist = computeDistance(robot_odom_->pose.pose.position,
                                    current_path_->poses.back().pose.position);

  if (goal_dist < goal_tolerance_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Goal reached. Stopping robot.");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // zero cmd
    return;
  }

  // Select lookahead point
  auto lookahead_point = findLookaheadPoint();
  if (!lookahead_point) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "No valid lookahead point found.");
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop for safety
    return;
  }

  // Compute and publish velocity command
  auto cmd_vel = computeVelocity(*lookahead_point);
  cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
  if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
    return std::nullopt;
  }

  // Find first pose at or beyond lookahead_distance_
  for (const auto &pose : current_path_->poses) {
    double distance = computeDistance(robot_odom_->pose.pose.position, pose.pose.position);
    if (distance >= lookahead_distance_) {
      return pose;
    }
  }

  // Fallback: return final waypoint (copy)
  return current_path_->poses.back();
}

geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
  geometry_msgs::msg::Twist cmd_vel;

  // Current robot pose
  auto robot_pose = robot_odom_->pose.pose;
  double rx = robot_pose.position.x;
  double ry = robot_pose.position.y;
  double ryaw = extractYaw(robot_pose.orientation);

  // Target pose
  double tx = target.pose.position.x;
  double ty = target.pose.position.y;

  // Vector to target & heading
  double dx = tx - rx;
  double dy = ty - ry;
  double target_yaw = std::atan2(dy, dx);

  // Normalize yaw error to [-pi, pi]
  double yaw_error = target_yaw - ryaw;
  while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
  while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

  // Distances
  double dist_to_target = std::hypot(dx, dy);
  double dist_to_goal = computeDistance(robot_pose.position, current_path_->poses.back().pose.position);

  // If at final goal -> stop
  if (dist_to_goal <= goal_tolerance_) {
    return cmd_vel; // zeros
  }

  // Linear command: proportional to distance to chosen target
  double linear_cmd = linear_kp_ * dist_to_target;

  // Slowdown when close to final goal
  if (dist_to_goal < slowdown_radius_) {
    double scale = std::max(0.0, dist_to_goal / slowdown_radius_);
    linear_cmd *= scale;
  }

  // clamp linear speed and ensure minimal nudge when needed
  linear_cmd = std::min(linear_cmd, linear_speed_);
  if (linear_cmd < min_linear_speed_ && dist_to_target > goal_tolerance_) {
    linear_cmd = min_linear_speed_;
  }

  // Angular: proportional and clamped
  double angular_cmd = angular_kp_ * yaw_error;
  if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
  if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;

  // If target is final waypoint, reduce angular aggressiveness
  const auto &final_pose = current_path_->poses.back().pose.position;
  if (std::fabs(tx - final_pose.x) < 1e-6 && std::fabs(ty - final_pose.y) < 1e-6) {
    angular_cmd *= 0.5;
  }

  // Reduce forward speed if yaw error is large
  const double yaw_threshold = 0.7; // radians (~40°)
  if (std::fabs(yaw_error) > yaw_threshold) {
    linear_cmd *= 0.4;
  }

  cmd_vel.linear.x = linear_cmd;
  cmd_vel.angular.z = angular_cmd;
  return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a,
                                    const geometry_msgs::msg::Point &b) const {
  return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) const {
  tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

// main
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
