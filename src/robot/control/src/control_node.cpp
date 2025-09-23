#include "control_node.hpp"


#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include <cmath>
using namespace std::chrono_literals;

ControlNode::ControlNode(): Node("control"), control_(robot::ControlCore(this->get_logger())) {
        lookahead_distance_ = 1.0;  // Lookahead distance
        goal_tolerance_ = 0.1;     // Distance to consider the goal reached
        linear_speed_ = 0.5;       // Constant forward speed
 
        // Subscribers and Publishers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10, [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });
 
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });
 
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
 
        // Timer
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });
    }


void ControlNode::controlLoop() {
    // Skip control if no path or odometry data is available
    if (!current_path_ || !robot_odom_) {
        return;
    }

    // Find the lookahead point
    auto lookahead_point = findLookaheadPoint();
    if (!lookahead_point) {
        return;  // No valid lookahead point found
    }

    // Compute velocity command
    auto cmd_vel = computeVelocity(*lookahead_point);

    // Publish the velocity command
    cmd_vel_pub_->publish(cmd_vel);
}

std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
        return std::nullopt;
    }

    auto robot_pose = robot_odom_->pose.pose;
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;

    // iterate through path to find first point at least lookahead_distance_ away
    for (const auto &pose_stamped : current_path_->poses) {
        double px = pose_stamped.pose.position.x;
        double py = pose_stamped.pose.position.y;
        double dist = computeDistance(robot_pose.position, pose_stamped.pose.position);
        if (dist >= lookahead_distance_) {
            return pose_stamped;
        }
    }

    // If none found above threshold, return the final goal if close enough
    auto &last = current_path_->poses.back();
    double final_dist = computeDistance(robot_pose.position, last.pose.position);
    if (final_dist <= goal_tolerance_) {
        return last;
    }
    return std::nullopt;
}
geometry_msgs::msg::Twist ControlNode::computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
    // TODO: Implement logic to compute velocity commands
    geometry_msgs::msg::Twist cmd_vel;

    // Extract robot’s current pose
    auto robot_pose = robot_odom_->pose.pose;
    double robot_x = robot_pose.position.x;
    double robot_y = robot_pose.position.y;
    double robot_yaw = extractYaw(robot_pose.orientation);

    // Target
    double tx = target.pose.position.x;
    double ty = target.pose.position.y;

    // Compute heading to target
    double dx = tx - robot_x;
    double dy = ty - robot_y;
    double target_yaw = std::atan2(dy, dx);

    double yaw_error = target_yaw - robot_yaw;
    // Normalize yaw_error to [-pi, pi]
    while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

    // Here: simple proportional steering
    double angular_speed = 2.0 * yaw_error;  // gain of 2.0, tune as needed

    // Linear speed could also drop when approaching goal or turning sharply
    double dist = std::sqrt(dx*dx + dy*dy);
    double linear_speed = linear_speed_; 
    if (dist < lookahead_distance_) {
        linear_speed = linear_speed_ * (dist / lookahead_distance_);
    }

    cmd_vel.linear.x = linear_speed;
    cmd_vel.angular.z = angular_speed;


    return cmd_vel;
}

double ControlNode::computeDistance(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b) {
    return std::hypot(a.x - b.x, a.y - b.y);
}

double ControlNode::extractYaw(const geometry_msgs::msg::Quaternion &quat) {
    tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ControlNode>());
  rclcpp::shutdown();
  return 0;
}
