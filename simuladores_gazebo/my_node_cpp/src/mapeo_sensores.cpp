#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include <algorithm>

// Webots-style colors
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"

class SensorMapper : public rclcpp::Node {
public:
    SensorMapper() : Node("mapeo_sensores") {
        auto qos = rclcpp::SensorDataQoS();
        
        sub_status = this->create_subscription<std_msgs::msg::String>(
            "/robot_status", 10, [this](const std_msgs::msg::String::SharedPtr msg) { status = msg->data; });
        
        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", qos, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                pos_x = msg->pose.pose.position.x;
                pos_y = msg->pose.pose.position.y;
                pos_z = msg->pose.pose.position.z;
            });

        auto qos_sensors = rclcpp::QoS(10);
        for (int i = 0; i < 8; ++i) {
            std::string topic = "/model/my_diffdrive_robot/link/chassis/sensor/ps" +
                               std::to_string(i) + "/lidar/scan";
            sub_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, qos_sensors, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty()) ps_val[i] = *std::min_element(msg->ranges.begin(), msg->ranges.end());
                });
            ps_val[i] = 10.0;
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&SensorMapper::draw, this));
        status = "INICIALIZANDO...";
        pos_x = 0; pos_y = 0; pos_z = 0;
        std::cout << "\033[2J" << std::flush;
    }

private:
    std::string get_bar(double val) {
        const double max_range = 3.0;
        int width = 12;
        double normalized = 1.0 - (std::min(val, max_range) / max_range);
        int fills = static_cast<int>(normalized * width);
        std::string bar = "[";
        for(int i=0; i<width; ++i) bar += (i < fills) ? "█" : "░";
        bar += "]";
        return bar;
    }

    std::string get_color(double val) {
        if (val < 0.4) return COLOR_RED;
        if (val < 0.8) return COLOR_YELLOW;
        return COLOR_GREEN;
    }

    void draw() {
        // Clear line and return to Home
        std::cout << "\033[H";
        
        std::cout << COLOR_CYAN << "╔══════════════════════════════════════════════════════════════════╗\033[K" << std::endl;
        std::cout << "║          DIAGNOSTICO DE SENSORES Y ESTADO (FSM P6)               ║\033[K" << std::endl;
        std::cout << "╚══════════════════════════════════════════════════════════════════╝" << COLOR_RESET << "\033[K" << std::endl;
        
        auto print_sensor = [this](int i, const std::string& label) {
            std::string c = get_color(ps_val[i]);
            std::string b = get_bar(ps_val[i]);
            char buf[128];
            // Format to exactly 32 chars visually
            snprintf(buf, sizeof(buf), "  ps%d %-7s: %s%5.2fm %s%s  ", i, label.c_str(), c.c_str(), ps_val[i], b.c_str(), COLOR_RESET);
            return std::string(buf);
        };

        std::cout << print_sensor(0, "(F-IZQ)") << "│" << print_sensor(7, "(F-DER)") << "\033[K" << std::endl;
        std::cout << print_sensor(1, "(L-45°)") << "│" << print_sensor(6, "(R-45°)") << "\033[K" << std::endl;
        std::cout << print_sensor(2, "(LAT-I)") << "│" << print_sensor(5, "(LAT-D)") << "\033[K" << std::endl;
        std::cout << print_sensor(3, "(B-IZQ)") << "│" << print_sensor(4, "(B-DER)") << "\033[K" << std::endl;
        
        std::cout << COLOR_BLUE << "╟──────────────────────────────────────────────────────────────────╢" << COLOR_RESET << "\033[K" << std::endl;
        
        // Robot Visualization
        std::cout << "               ^   " << (ps_val[0]<0.8?"\033[31mBLOCK\033[0m":" \033[32mOK \033[0m ") << " -- " << (ps_val[7]<0.8?"\033[31mBLOCK\033[0m":" \033[32mOK \033[0m ") << "   ^\033[K" << std::endl;
        std::cout << "               |      +------------------+      |\033[K" << std::endl;
        std::cout << "               /       |      CHASSIS     |      \\\033[K" << std::endl;
        std::cout << "              <---------|       2.0 m      |--------->\033[K" << std::endl;
        std::cout << "               \\       |       1.0 m      |      /\033[K" << std::endl;
        std::cout << "               |      +------------------+      |\033[K" << std::endl;
        std::cout << "               v                                v\033[K" << std::endl;

        std::cout << COLOR_BLUE << "╟──────────────────────────────────────────────────────────────────╢" << COLOR_RESET << "\033[K" << std::endl;
        
        std::cout << COLOR_WHITE << "  [ TELEMETRIA ]\033[K" << COLOR_RESET << std::endl;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "   X: " << std::setw(8) << pos_x << " m  │  Y: " << std::setw(8) << pos_y << " m  │  Z: " << std::setw(8) << pos_z << " m\033[K" << std::endl;
        
        std::cout << std::endl;
        std::cout << COLOR_YELLOW << "  [ LOGICA FSM ]\033[K" << COLOR_RESET << std::endl;
        std::cout << "   >> ESTADO: " << COLOR_WHITE << status << COLOR_RESET << "\033[K" << std::endl;
        
        std::cout << COLOR_CYAN << "╚══════════════════════════════════════════════════════════════════╝" << COLOR_RESET << "\033[K" << std::endl;
        std::cout << "\033[J" << std::flush;
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_status;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ps[8];
    rclcpp::TimerBase::SharedPtr timer;
    
    double pos_x, pos_y, pos_z, ps_val[8];
    std::string status;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SensorMapper>());
    rclcpp::shutdown();
    return 0;
}
