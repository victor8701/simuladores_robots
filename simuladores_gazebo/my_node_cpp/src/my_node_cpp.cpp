#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <tf2/utils.h>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>

using namespace std::placeholders;

// Global Colors
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_RED     "\033[31m"
#define COLOR_MAGENTA "\033[35m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_WHITE   "\033[37m"

class HolonomicNode : public rclcpp::Node {
public:
    HolonomicNode() : Node("my_node_cpp") {
        print_colored("=== HOLONOMIC MANHATTAN CONTROLLER READY ===", COLOR_CYAN);

        auto qos = rclcpp::SensorDataQoS();

        pub = this->create_publisher<geometry_msgs::msg::Twist>("/model/my_diffdrive_robot/cmd_vel", 10);
        
        // --- POSITION SOURCES ---
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", qos, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) { update_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, "ODOM_STD"); });

        sub_pose = this->create_subscription<tf2_msgs::msg::TFMessage>(
            "/model/my_diffdrive_robot/pose", qos, 
            [this](const tf2_msgs::msg::TFMessage::SharedPtr msg) {
                for(const auto& t : msg->transforms) {
                    if(t.child_frame_id == "my_diffdrive_robot" || t.child_frame_id == "chassis") {
                        update_pos(t.transform.translation.x, t.transform.translation.y, "TF_POSE");
                    }
                }
            });

        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu", qos, 
            [this](const sensor_msgs::msg::Imu::SharedPtr msg) {
                tf2::Quaternion q(msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
                current_yaw = tf2::getYaw(q);
            });

        // --- SENSORS PS0-PS7 ---
        for (int i = 0; i < 8; ++i) {
            std::string topic = "/ps" + std::to_string(i);
            subs_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, qos, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty()) {
                        // Use the minimum range from the samples for safety
                        float min_r = 10.0;
                        for(auto r : msg->ranges) if(r < min_r) min_r = r;
                        ps_values[i] = min_r;
                    }
                });
            ps_values[i] = 3.0; 
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&HolonomicNode::control_loop, this));
        
        initialized_pos = false;
        goal_reached = false;
        last_state = "";
        
        // Stall detection
        last_pos_check[0] = 0; last_pos_check[1] = 0;
        stall_counter = 0;
    }

private:
    void print_colored(const std::string& msg, const std::string& color) {
        std::cout << color << msg << COLOR_RESET << std::endl;
    }

    void update_pos(double x, double y, const std::string& source) {
        current_pos[0] = x;
        current_pos[1] = y;
        
        if (!initialized_pos) {
            active_source = source;
            calculate_goal();
            initialized_pos = true;
            print_colored("OK: Ubicación detectada (" + active_source + ").", COLOR_GREEN);
        }
    }

    void calculate_goal() {
        position_goal[0] = (current_pos[0] < 6.0) ? 10.5 : 1.5;
        position_goal[1] = (current_pos[1] < 6.0) ? 10.5 : 1.5;
        RCLCPP_INFO(this->get_logger(), "Meta: [%.2f, %.2f]", position_goal[0], position_goal[1]);
    }

    void control_loop() {
        if (!initialized_pos) {
            static int count = 0;
            if (count++ % 20 == 0) print_colored("ESPERANDO UBICACION...", COLOR_RED);
            return;
        }
        
        if (goal_reached) return;

        // Stall detection (every 1 second approx)
        static int stall_tick = 0;
        if (stall_tick++ % 10 == 0) {
            double dp = std::sqrt(std::pow(current_pos[0]-last_pos_check[0], 2) + std::pow(current_pos[1]-last_pos_check[1], 2));
            if (dp < 0.05) stall_counter++;
            else stall_counter = 0;
            last_pos_check[0] = current_pos[0];
            last_pos_check[1] = current_pos[1];
        }

        // Telemetry
        std::stringstream ss;
        ss << std::fixed << std::setprecision(2);
        ss << "POS:[" << current_pos[0] << "," << current_pos[1] << "] | PS:[";
        for (int i = 0; i < 8; ++i) ss << (ps_values[i] > 2.9 ? "OK" : std::to_string(ps_values[i]).substr(0,4)) << (i==7?"]":",");
        print_colored(ss.str(), COLOR_YELLOW);

        double errX = position_goal[0] - current_pos[0];
        double errY = position_goal[1] - current_pos[1];
        double dist = std::sqrt(errX*errX + errY*errY);

        if (dist < 0.3) {
            goal_reached = true;
            stop_robot();
            print_colored("¡OBJETIVO ALCANZADO!", COLOR_GREEN);
            return;
        }

        auto twist = geometry_msgs::msg::Twist();
        
        // --- IMPROVED COLLISION SAFETY ---
        bool wall_front = (ps_values[0] < 1.0 || ps_values[7] < 1.0 || ps_values[1] < 0.6 || ps_values[6] < 0.6);
        bool wall_back  = (ps_values[3] < 0.8 || ps_values[4] < 0.8);
        bool wall_left  = (ps_values[2] < 0.7); 
        bool wall_right = (ps_values[5] < 0.7);
        
        // Forced evasion if stalled
        bool is_stalled = (stall_counter > 3); 

        bool moving_X = false;
        if (std::abs(errX) > 0.3) {
            if (errX > 0) { // Moving X+
                if (wall_front || (is_stalled && errX > 0)) { 
                    twist.linear.x = -0.4; 
                    log_state(is_stalled ? "STALL DETECTADO (X+) -> RETROCEDIENDO" : "EVADIENDO FRONTAL", COLOR_RED);
                } else { 
                    twist.linear.x = 0.6; 
                    log_state("MOVIENDO X+", COLOR_WHITE); 
                    moving_X = true; 
                }
            } else { // Moving X-
                if (wall_back || (is_stalled && errX < 0)) { 
                    twist.linear.x = 0.4; 
                    log_state(is_stalled ? "STALL DETECTADO (X-) -> RETROCEDIENDO" : "EVADIENDO TRASERO", COLOR_RED);
                } else { 
                    twist.linear.x = -0.6; 
                    log_state("MOVIENDO X-", COLOR_WHITE); 
                    moving_X = true; 
                }
            }
        }

        if (!moving_X && std::abs(errY) > 0.3) {
            if (errY > 0) { // Moving Y+
                if (wall_left || (is_stalled && errY > 0)) { 
                    twist.linear.y = -0.4; 
                    log_state("EVADIENDO LATERAL IZQ", COLOR_RED); 
                } else { 
                    twist.linear.y = 0.6; 
                    log_state("MOVIENDO Y+", COLOR_WHITE); 
                }
            } else { // Moving Y-
                if (wall_right || (is_stalled && errY < 0)) { 
                    twist.linear.y = 0.4; 
                    log_state("EVADIENDO LATERAL DER", COLOR_RED); 
                } else { 
                    twist.linear.y = -0.6; 
                    log_state("MOVIENDO Y-", COLOR_WHITE); 
                }
            }
        }

        twist.angular.z = 0.0;
        pub->publish(twist);
    }

    void log_state(const std::string& state, const std::string& color) {
        if (state != last_state) {
            print_colored(">>> " + state, color);
            last_state = state;
        }
    }

    void stop_robot() {
        auto twist = geometry_msgs::msg::Twist();
        pub->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_pose;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subs_ps[8];
    rclcpp::TimerBase::SharedPtr timer;

    double current_pos[2];
    double current_yaw;
    double position_goal[2];
    double ps_values[8];
    bool initialized_pos;
    bool goal_reached;
    std::string active_source;
    std::string last_state;
    
    // Stall detection
    double last_pos_check[2];
    int stall_counter;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<HolonomicNode>());
    rclcpp::shutdown();
    return 0;
}