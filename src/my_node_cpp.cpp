#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>

using namespace std::placeholders;

// Logic ported from e-puck_avoid_obstacles_VMP.cpp
// Target: Diagonal corner from starting position

#define MAX_SPEED 0.5
#define TURN_SPEED 0.3
#define WALL_THRESHOLD 0.3
#define FRONT_WALL_THRESHOLD 0.4
#define COLLISION_THRESHOLD 0.15
#define GOAL_RADIUS 0.3
#define CORRECTION_FACTOR 0.3

class MyNodeCpp : public rclcpp::Node {
public:
    MyNodeCpp() : Node("my_node_cpp") {
        RCLCPP_INFO(this->get_logger(), "Node initialized with Webots behavior.");

        pub = this->create_publisher<geometry_msgs::msg::Twist>("/model/my_diffdrive_robot/cmd_vel", 10);
        
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", 10, std::bind(&MyNodeCpp::on_odom, this, std::placeholders::_1));

        for (int i = 0; i < 8; ++i) {
            std::string topic = "/model/my_diffdrive_robot/link/chassis/sensor/ps" + std::to_string(i) + "/scan";
            subs_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, 10, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty()) {
                        ps_values[i] = msg->ranges[0];
                    }
                });
            ps_values[i] = 1.0; // Default far
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&MyNodeCpp::control_loop, this));
        
        initialized_pos = false;
        goal_reached = false;
    }

private:
    void on_odom(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos[0] = msg->pose.pose.position.x;
        current_pos[1] = msg->pose.pose.position.y;
        
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        current_yaw = tf2::getYaw(q);

        if (!initialized_pos) {
            calculate_goal();
            initialized_pos = true;
        }
    }

    void calculate_goal() {
        // Simple logic to find diagonal corner in 12x12 world
        position_goal[0] = (current_pos[0] < 6.0) ? 10.9 : 1.1;
        position_goal[1] = (current_pos[1] < 6.0) ? 10.9 : 1.1;
        RCLCPP_INFO(this->get_logger(), "Goal set to: [%.2f, %.2f]", position_goal[0], position_goal[1]);
    }

    void control_loop() {
        if (!initialized_pos || goal_reached) return;

        double dist_to_goal = std::sqrt(std::pow(position_goal[0] - current_pos[0], 2) + 
                                       std::pow(position_goal[1] - current_pos[1], 2));
        
        if (dist_to_goal < GOAL_RADIUS) {
            goal_reached = true;
            stop_robot();
            RCLCPP_INFO(this->get_logger(), "Goal reached!");
            return;
        }

        auto twist = geometry_msgs::msg::Twist();
        
        // Basic Port of Webots Logic
        bool wall_front = (ps_values[0] < FRONT_WALL_THRESHOLD || ps_values[7] < FRONT_WALL_THRESHOLD);
        bool wall_right = (ps_values[2] < WALL_THRESHOLD || ps_values[1] < WALL_THRESHOLD);
        bool wall_left = (ps_values[5] < WALL_THRESHOLD || ps_values[6] < WALL_THRESHOLD);

        if (wall_front) {
            // Turn away from wall
            twist.angular.z = (current_pos[1] < position_goal[1]) ? TURN_SPEED : -TURN_SPEED;
            twist.linear.x = 0.0;
        } else if (wall_right || wall_left) {
            // Wall following behavior
            twist.linear.x = MAX_SPEED;
            // Align with cardinal directions (Manhattan logic from original)
            double target_yaw = round(current_yaw / (M_PI/2.0)) * (M_PI/2.0);
            double yaw_err = target_yaw - current_yaw;
            twist.angular.z = yaw_err * CORRECTION_FACTOR;
        } else {
            // Free navigation to goal
            double angle_to_goal = std::atan2(position_goal[1] - current_pos[1], position_goal[0] - current_pos[0]);
            double yaw_err = angle_to_goal - current_yaw;
            while (yaw_err > M_PI) yaw_err -= 2*M_PI;
            while (yaw_err < -M_PI) yaw_err += 2*M_PI;

            if (std::abs(yaw_err) > 0.2) {
                twist.angular.z = (yaw_err > 0) ? TURN_SPEED : -TURN_SPEED;
                twist.linear.x = 0.1;
            } else {
                twist.linear.x = MAX_SPEED;
                twist.angular.z = yaw_err * 0.5;
            }
        }

        pub->publish(twist);
    }

    void stop_robot() {
        auto twist = geometry_msgs::msg::Twist();
        pub->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs_ps[8];
    rclcpp::TimerBase::SharedPtr timer;

    double current_pos[2];
    double current_yaw;
    double position_goal[2];
    double ps_values[8];
    bool initialized_pos;
    bool goal_reached;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MyNodeCpp>());
    rclcpp::shutdown();
    return 0;
}