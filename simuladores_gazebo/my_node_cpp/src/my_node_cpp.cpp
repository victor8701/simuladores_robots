#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

using namespace std::placeholders;

/**
 * Port of e-puck_avoid_obstacles_VMP.cpp to ROS 2 / Gazebo
 * Implements Manhattan-style movement, Wall-Following, and Cardinal Alignment.
 */

// Velocities
#define MAX_SPEED 0.4
#define TURN_SPEED 0.3

// Thresholds (Calibrated from ps 0..4096 to meters)
// Webots: WALL_THRESHOLD 90 -> approx 0.15m
// Webots: FRONT_WALL_THRESHOLD 190 -> approx 0.1m
#define WALL_THRESHOLD_M 0.25
#define FRONT_WALL_THRESHOLD_M 0.28
#define GOAL_RADIUS 0.25
#define CORRECTION_FACTOR 0.8

class MyNodeCpp : public rclcpp::Node {
public:
    MyNodeCpp() : Node("my_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "Node initialized with Manhattan logic (Webots parity).");

        pub = this->create_publisher<geometry_msgs::msg::Twist>("/model/my_diffdrive_robot/cmd_vel", 10);
        
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", 10, std::bind(&MyNodeCpp::on_odom, this, _1));

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/model/my_diffdrive_robot/link/chassis/sensor/imu_sensor/imu", 10, std::bind(&MyNodeCpp::on_imu, this, _1));

        for (int i = 0; i < 8; ++i) {
            std::string topic = "/model/my_diffdrive_robot/link/chassis/sensor/ps" + std::to_string(i) + "/scan";
            subs_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, 10, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty()) {
                        ps_values[i] = msg->ranges[0];
                    }
                });
            ps_values[i] = 1.0; 
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&MyNodeCpp::control_loop, this));
        
        initialized_pos = false;
        goal_reached = false;
        following_wall = false;
        wall_on_right = true;
        current_yaw = 0.0;
    }

private:
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos[0] = msg->pose.pose.position.x;
        current_pos[1] = msg->pose.pose.position.y;
        
        if (!initialized_pos) {
            calculate_goal();
            initialized_pos = true;
        }
    }

    void on_imu(const sensor_msgs::msg::Imu::SharedPtr msg) {
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        current_yaw = tf2::getYaw(q);
        // Normalize 0-2PI
        if (current_yaw < 0) current_yaw += 2 * M_PI;
    }

    void calculate_goal() {
        // Target: diagonal corner from starting position
        position_goal[0] = (current_pos[0] < 6.0) ? 10.9 : 1.1;
        position_goal[1] = (current_pos[1] < 6.0) ? 10.9 : 1.1;
        RCLCPP_INFO(this->get_logger(), "Targeting corner: [%.2f, %.2f]", position_goal[0], position_goal[1]);
    }

    void control_loop() {
        if (!initialized_pos || goal_reached) return;

        double dist_to_goal = std::sqrt(std::pow(position_goal[0] - current_pos[0], 2) + 
                                       std::pow(position_goal[1] - current_pos[1], 2));
        
        if (dist_to_goal < GOAL_RADIUS) {
            goal_reached = true;
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Goal reached! Manhattan path complete.");
            return;
        }

        double vel_left = 0, vel_right = 0;
        
        // Sensor states
        bool wall_front = (ps_values[0] < FRONT_WALL_THRESHOLD_M || ps_values[7] < FRONT_WALL_THRESHOLD_M);
        bool wall_right = (ps_values[2] < WALL_THRESHOLD_M || ps_values[1] < WALL_THRESHOLD_M);
        bool wall_left = (ps_values[5] < WALL_THRESHOLD_M || ps_values[6] < WALL_THRESHOLD_M);

        if (wall_front || wall_right || wall_left) {
            following_wall = true;
            if (wall_right) wall_on_right = true;
            else if (wall_left) wall_on_right = false;
        }

        if (following_wall) {
            // Wall following with cardinal alignment
            if (wall_front) {
                // Corner detection: rotate
                if (wall_on_right) { rotate(-TURN_SPEED, vel_left, vel_right); }
                else { rotate(TURN_SPEED, vel_left, vel_right); }
            } else if ((wall_on_right && wall_right) || (!wall_on_right && wall_left)) {
                // Keep moving forward aligned to nearest axis
                vel_left = MAX_SPEED;
                vel_right = MAX_SPEED;
                apply_cardinal_correction(vel_left, vel_right);
            } else {
                // Lost wall: search again or exit to free navigation
                following_wall = false;
            }
        } else {
            // Manhattan Navigation towards goal
            double errX = position_goal[0] - current_pos[0];
            double errY = position_goal[1] - current_pos[1];
            
            double target_yaw = 0;
            // Precise logic from line 252: moveInX = (abs(errX) < abs(errY))
            // To match diagonal corners, this forces the robot to prioritize one axis until close
            if (std::abs(errX) < std::abs(errY)) {
                target_yaw = (errX > 0) ? 0.0 : M_PI;
            } else {
                target_yaw = (errY > 0) ? M_PI/2.0 : 3*M_PI/2.0;
            }

            double yaw_err = target_yaw - current_yaw;
            while (yaw_err > M_PI) yaw_err -= 2*M_PI;
            while (yaw_err < -M_PI) yaw_err += 2*M_PI;

            if (std::abs(yaw_err) > 0.1) {
                rotate((yaw_err > 0) ? TURN_SPEED : -TURN_SPEED, vel_left, vel_right);
            } else {
                vel_left = MAX_SPEED;
                vel_right = MAX_SPEED;
                // Strong alignment
                vel_left -= yaw_err * CORRECTION_FACTOR;
                vel_right += yaw_err * CORRECTION_FACTOR;
            }
        }

        publish_twist(vel_left, vel_right);
    }

    void apply_cardinal_correction(double& vL, double& vR) {
        // Find nearest cardinal (0, 90, 180, 270)
        double yaw_deg = current_yaw * 180.0 / M_PI;
        double nearest_deg = std::round(yaw_deg / 90.0) * 90.0;
        double error_deg = nearest_deg - yaw_deg;
        while (error_deg > 180) error_deg -= 360;
        while (error_deg < -180) error_deg += 360;

        double correction = (error_deg * M_PI / 180.0) * CORRECTION_FACTOR;
        vL -= correction;
        vR += correction;
    }

    void rotate(double angular_vel, double& vL, double& vR) {
        vL = -angular_vel; // simplified diff drive model scaling
        vR = angular_vel;
    }

    void publish_twist(double vL, double vR) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = (vL + vR) / 2.0;
        msg.angular.z = (vR - vL) / 0.5; // Adjusted factor for rotation feel
        pub->publish(msg);
    }

    void stop_robot() {
        publish_twist(0, 0);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs_ps[8];
    rclcpp::TimerBase::SharedPtr timer;

    double current_pos[2];
    double current_yaw;
    double position_goal[2];
    double ps_values[8];
    bool initialized_pos;
    bool goal_reached;
    bool following_wall;
    bool wall_on_right;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNodeCpp>());
    rclcpp::shutdown();
    return 0;
}