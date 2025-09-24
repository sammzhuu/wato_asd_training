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
        goal_tolerance_     = this->declare_parameter("goal_tolerance", 0.1);
        linear_speed_       = this->declare_parameter("linear_speed", 0.5);
        linear_kp_          = this->declare_parameter("linear_kp", 0.5);
        angular_kp_         = this->declare_parameter("angular_kp", 1.0);

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

    std::optional<geometry_msgs::msg::PoseStamped> findLookaheadPoint() {
        for (auto &pose : current_path_->poses) {
            double distance = computeDistance(
                robot_odom_->pose.pose.position, pose.pose.position);
            if (distance >= lookahead_distance_) {
                return pose;
            }
        }
        return std::nullopt;
    }

    geometry_msgs::msg::Twist computeVelocity(const geometry_msgs::msg::PoseStamped &target) {
        geometry_msgs::msg::Twist cmd_vel;

        // Current robot pose
        auto robot_pose = robot_odom_->pose.pose;
        double robot_x = robot_pose.position.x;
        double robot_y = robot_pose.position.y;
        double robot_yaw = extractYaw(robot_pose.orientation);

        // Target
        double tx = target.pose.position.x;
        double ty = target.pose.position.y;

        // Vector to target
        double dx = tx - robot_x;
        double dy = ty - robot_y;
        double target_yaw = std::atan2(dy, dx);

        // Heading error
        double yaw_error = target_yaw - robot_yaw;
        while (yaw_error > M_PI) yaw_error -= 2 * M_PI;
        while (yaw_error < -M_PI) yaw_error += 2 * M_PI;

        // Distance to target
        double dist = std::sqrt(dx * dx + dy * dy);

        // --- Control law ---
        // Linear speed decreases as we approach goal
        double goal_dist = computeDistance(robot_pose.position, current_path_->poses.back().pose.position);
        cmd_vel.linear.x = std::min(linear_speed_, linear_kp_ * goal_dist);

        // Angular speed proportional to yaw error
        cmd_vel.angular.z = angular_kp_ * yaw_error;

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
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
