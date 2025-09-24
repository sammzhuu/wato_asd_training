#include "control_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <optional>

class ControlNode : public rclcpp::Node {
public:
    ControlNode() : Node("pure_pursuit_controller") {
        // Parameters (tunable via ROS2 params)
        lookahead_distance_ = this->declare_parameter("lookahead_distance", 1.0);
        goal_tolerance_     = this->declare_parameter("goal_tolerance", 0.05);
        linear_speed_       = this->declare_parameter("linear_speed", 0.5);
        linear_kp_          = this->declare_parameter("linear_kp", 0.5);
        angular_kp_         = this->declare_parameter("angular_kp", 1.0);
        min_linear_speed_   = 0.05;   // ensure we still inch forward
        max_angular_speed_  = 1.0;    // rad/s clamp
        slowdown_radius_    = 0.5;    // start slowing before final

        // Subscribers
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) { current_path_ = msg; });

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom/filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { robot_odom_ = msg; });

        // Publisher
        cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Timer: 10 Hz control loop
        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), [this]() { controlLoop(); });

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Controller initialized.");
    }

private:
    void controlLoop() {
        if (!current_path_ || !robot_odom_ || current_path_->poses.empty()) {
            return;
        }

        // --- Check goal condition ---
        double goal_dist = computeDistance(
            robot_odom_->pose.pose.position,
            current_path_->poses.back().pose.position);

        if (goal_dist < goal_tolerance_) {
            RCLCPP_INFO_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "Goal reached. Stopping robot.");
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // zero cmd
            return;
        }

        // --- Select lookahead point ---
        auto lookahead_point = findLookaheadPoint();
        if (!lookahead_point) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 2000, "No valid lookahead point found.");
            cmd_vel_pub_->publish(geometry_msgs::msg::Twist());  // stop for safety
            return;
        }

        // --- Compute velocity command ---
        auto cmd_vel = computeVelocity(*lookahead_point);

        // --- Publish ---
        cmd_vel_pub_->publish(cmd_vel);
    }

    std::optional<geometry_msgs::msg::PoseStamped> ControlNode::findLookaheadPoint() {
    if (!current_path_ || current_path_->poses.empty() || !robot_odom_) {
        return std::nullopt;
    }

    // first try to find a point >= lookahead_distance_
    for (const auto &pose : current_path_->poses) {
        double distance = computeDistance(robot_odom_->pose.pose.position, pose.pose.position);
        if (distance >= lookahead_distance_) {
            return pose;
        }
    }

    // If none found, FALL BACK to the final waypoint (important!)
        return current_path_->poses.back();
    }

    // --- computeVelocity ---
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

        // Vector to target
        double dx = tx - rx;
        double dy = ty - ry;
        double target_yaw = std::atan2(dy, dx);

        // Yaw error normalized
        double yaw_error = target_yaw - ryaw;
        while (yaw_error > M_PI)  yaw_error -= 2.0 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

        // Distances
        double dist_to_target = std::hypot(dx, dy);
        double dist_to_goal   = computeDistance(robot_pose.position, current_path_->poses.back().pose.position);

        // If final goal reached (center within tolerance) -> zero velocities
        if (dist_to_goal <= goal_tolerance_) {
            return cmd_vel; // zeros
        }

        // Linear: proportional to distance *to the chosen target* (not to some other goal)
        double linear_cmd = linear_kp_ * dist_to_target;

        // When close to the final goal, apply extra slowdown factor (smooth arrival)
        if (dist_to_goal < slowdown_radius_) {
            // scale down based on how close to final
            double scale = std::max(0.0, dist_to_goal / slowdown_radius_);
            linear_cmd *= scale;
        }

        // clamp and enforce minimum so we actually close the last bit
        linear_cmd = std::min(linear_cmd, linear_speed_);
        if (linear_cmd < min_linear_speed_ && dist_to_target > goal_tolerance_) {
            // allow a small nudge forward if we're not yet inside goal tolerance
            linear_cmd = min_linear_speed_;
        }

        // Angular: proportional but clamped
        double angular_cmd = angular_kp_ * yaw_error;
        if (angular_cmd > max_angular_speed_) angular_cmd = max_angular_speed_;
        if (angular_cmd < -max_angular_speed_) angular_cmd = -max_angular_speed_;

        // If the target is the final waypoint, reduce angular command to avoid spinning in place
        if (target.pose.position.x == current_path_->poses.back().pose.position.x &&
            target.pose.position.y == current_path_->poses.back().pose.position.y) {
            // allow small heading correction but don't rotate aggressively
            angular_cmd *= 0.5;
        }

        // If yaw error is large, consider reducing linear speed (optional safety)
        const double yaw_threshold = 0.7; // radians (~40°)
        if (std::fabs(yaw_error) > yaw_threshold) {
            // reduce forward speed while making a large turn
            linear_cmd *= 0.4;
        }

        cmd_vel.linear.x  = linear_cmd;
        cmd_vel.angular.z = angular_cmd;
        return cmd_vel;
    }

    double computeDistance(const geometry_msgs::msg::Point &a,
                           const geometry_msgs::msg::Point &b) {
        return std::hypot(a.x - b.x, a.y - b.y);
    }

    double extractYaw(const geometry_msgs::msg::Quaternion &quat) {
        tf2::Quaternion q(quat.x, quat.y, quat.z, quat.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    // --- ROS interfaces ---
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // --- Data ---
    nav_msgs::msg::Path::SharedPtr current_path_;
    nav_msgs::msg::Odometry::SharedPtr robot_odom_;

    // --- Parameters ---
    double lookahead_distance_;
    double goal_tolerance_;
    double linear_speed_;
    double linear_kp_;
    double angular_kp_;
    double min_linear_speed_;
    double max_angular_speed_;
    double slowdown_radius_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
