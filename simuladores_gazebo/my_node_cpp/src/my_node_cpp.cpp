#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <vector>
#include <string>
#include <iomanip>
#include <iostream>
#include <algorithm>

using namespace std::placeholders;

// --- CONFIGURABLE PARAMETERS ---
#define OVERSHOOT_DIST 0.85      
#define WALL_FRONT_LIMIT 0.75    
#define WALL_SIDE_LIMIT 0.85     
#define ROBOT_SPEED 0.6          
#define YAW_KP 2.5               

enum State { LIBRE, SIGUIENDO_PARED, INT_CORNER, OVERSHOOT, META_ALCANZADA };
enum WallType { NONE, FRONT, BACK, LEFT, RIGHT };
enum DirType { STOP, FORWARD, BACKWARD, MOVE_LEFT, MOVE_RIGHT };

class ManhattanController : public rclcpp::Node {
public:
    ManhattanController() : Node("my_node_cpp") {
        auto qos = rclcpp::SensorDataQoS();
        pub_vel = this->create_publisher<geometry_msgs::msg::Twist>("/model/my_diffdrive_robot/cmd_vel", 10);
        pub_state = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", qos, 
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                update_state(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
            });
        
        for (int i = 0; i < 8; ++i) {
            std::string topic = "/ps" + std::to_string(i);
            sub_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, qos, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty()) ps_val[i] = *std::min_element(msg->ranges.begin(), msg->ranges.end());
                });
            ps_val[i] = 10.0;
        }

        timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&ManhattanController::control_loop, this));
        
        initialized = false;
        goal_reached = false;
        current_state = LIBRE;
        hugged_wall = NONE;
        travel_dir = STOP;
        yaw = 0.0;
        
        std::cout << "\033[1;36m[ManhattanController] Nodo Iniciado. FSM P6 (Memory) Online.\033[0m" << std::endl;
    }

private:
    void update_state(double x, double y, double qz, double qw) {
        pos_x = x; pos_y = y;
        yaw = 2.0 * std::atan2(qz, qw);
        if (!initialized) {
            goal_x = (pos_x < 6.0) ? 10.5 : 1.5;
            goal_y = (pos_y < 6.0) ? 10.5 : 1.5;
            initialized = true;
        }
    }

    void control_loop() {
        if (!initialized || goal_reached) return;

        double errX = goal_x - pos_x;
        double errY = goal_y - pos_y;
        
        if (std::sqrt(errX*errX + errY*errY) < 0.35) {
            goal_reached = true; stop(); 
            publish_status("META ALCANZADA");
            return;
        }

        bool wall_F = (ps_val[0] < WALL_FRONT_LIMIT || ps_val[7] < WALL_FRONT_LIMIT);
        bool wall_B = (ps_val[3] < WALL_FRONT_LIMIT || ps_val[4] < WALL_FRONT_LIMIT);
        bool wall_L = (ps_val[2] < WALL_SIDE_LIMIT);
        bool wall_R = (ps_val[5] < WALL_SIDE_LIMIT);

        auto twist = geometry_msgs::msg::Twist();
        twist.angular.z = -YAW_KP * yaw;

        double vx_w = 0, vy_w = 0;

        switch (current_state) {
            case LIBRE:
                // Move along axis with SMALLEST error
                if (std::abs(errX) < std::abs(errY) && std::abs(errX) > 0.3) {
                    vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                    travel_dir = (errX > 0) ? FORWARD : BACKWARD;
                } else if (std::abs(errY) > 0.3) {
                    vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                    travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
                }

                if (wall_F) { current_state = SIGUIENDO_PARED; hugged_wall = FRONT; }
                else if (wall_B) { current_state = SIGUIENDO_PARED; hugged_wall = BACK; }
                else if (wall_L) { current_state = SIGUIENDO_PARED; hugged_wall = LEFT; }
                else if (wall_R) { current_state = SIGUIENDO_PARED; hugged_wall = RIGHT; }
                publish_status("LIBRE (Dir: " + dir_to_str(travel_dir) + ")");
                break;

            case SIGUIENDO_PARED:
                // Detect interior corner
                if ((wall_F && wall_L) || (wall_F && wall_R) || (wall_B && wall_L) || (wall_B && wall_R)) {
                    current_state = INT_CORNER; break;
                }
                // Detect exterior corner (hugged wall disappears)
                if ((hugged_wall == FRONT && !wall_F) || (hugged_wall == BACK && !wall_B) ||
                    (hugged_wall == LEFT && !wall_L) || (hugged_wall == RIGHT && !wall_R)) {
                    start_overshoot(last_vx, last_vy); break;
                }

                // Normal following: ignore error on hugged axis
                if (hugged_wall == FRONT || hugged_wall == BACK) {
                    if (std::abs(errY) > 0.3) {
                        vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
                    }
                } else {
                    if (std::abs(errX) > 0.3) {
                        vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errX > 0) ? FORWARD : BACKWARD;
                    }
                }
                publish_status("SIGUIENDO PARED " + wall_to_str(hugged_wall));
                break;

            case INT_CORNER:
                if (wall_F && wall_L) { // SUP-IZQ
                    publish_status("ESQUINA INT. SUP-IZQ");
                    if (hugged_wall == LEFT) { vx_w = 0; vy_w = -ROBOT_SPEED; } // Move Right
                    else { vx_w = -ROBOT_SPEED; vy_w = 0; } // Move Back
                } else if (wall_F && wall_R) { // SUP-DER
                    publish_status("ESQUINA INT. SUP-DER");
                    if (hugged_wall == RIGHT) { vx_w = 0; vy_w = ROBOT_SPEED; } // Move Left
                    else { vx_w = -ROBOT_SPEED; vy_w = 0; } // Move Back
                } else if (wall_B && wall_L) { // INF-IZQ
                    publish_status("ESQUINA INT. INF-IZQ");
                    if (hugged_wall == LEFT) { vx_w = 0; vy_w = -ROBOT_SPEED; }
                    else { vx_w = ROBOT_SPEED; vy_w = 0; } // Move Front
                } else if (wall_B && wall_R) { // INF-DER
                    publish_status("ESQUINA INT. INF-DER");
                    if (hugged_wall == RIGHT) { vx_w = 0; vy_w = ROBOT_SPEED; }
                    else { vx_w = ROBOT_SPEED; vy_w = 0; }
                }

                if (!wall_F && !wall_B && !wall_L && !wall_R) current_state = LIBRE;
                break;

            case OVERSHOOT:
                double d = std::sqrt(std::pow(pos_x - over_start_x, 2) + std::pow(pos_y - over_start_y, 2));
                if (d >= OVERSHOOT_DIST) {
                    publish_status("GIRO TRAS OVERSHOOT (" + wall_to_str(hugged_wall) + ")");
                    // Decision based on memory
                    if (hugged_wall == FRONT) { 
                        // Was hugging Front, Front gone. Move forward to recapture it on side.
                        vx_w = (travel_dir == MOVE_LEFT) ? ROBOT_SPEED : ROBOT_SPEED; 
                        vy_w = 0; 
                    } else if (hugged_wall == LEFT) {
                        // Was hugging Left, Left gone. Move Left to recapture.
                        vx_w = 0; vy_w = ROBOT_SPEED;
                    } // ... etc
                    current_state = LIBRE; 
                } else {
                    vx_w = over_vx; vy_w = over_vy;
                    publish_status("OVERSHOOT: Limpiando esquina...");
                }
                break;
        }

        last_vx = vx_w; last_vy = vy_w;
        twist.linear.x = vx_w * std::cos(yaw) + vy_w * std::sin(yaw);
        twist.linear.y = -vx_w * std::sin(yaw) + vy_w * std::cos(yaw);
        pub_vel->publish(twist);
    }

    void start_overshoot(double vx, double vy) {
        current_state = OVERSHOOT; over_start_x = pos_x; over_start_y = pos_y;
        over_vx = vx; over_vy = vy;
    }

    std::string dir_to_str(DirType d) {
        if(d==FORWARD) return "ADELANTE"; if(d==BACKWARD) return "ATRAS";
        if(d==MOVE_LEFT) return "IZQUIERDA"; if(d==MOVE_RIGHT) return "DERECHA";
        return "PARADO";
    }
    std::string wall_to_str(WallType w) {
        if(w==FRONT) return "FRONTAL"; if(w==BACK) return "INFERIOR";
        if(w==LEFT) return "IZQUIERDA"; if(w==RIGHT) return "DERECHA";
        return "NINGUNA";
    }
    void publish_status(const std::string& msg) {
        if (msg != last_status) {
            auto s = std_msgs::msg::String(); s.data = msg;
            pub_state->publish(s); std::cout << ">>> " << msg << std::endl;
            last_status = msg;
        }
    }
    void stop() { pub_vel->publish(geometry_msgs::msg::Twist()); }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ps[8];
    rclcpp::TimerBase::SharedPtr timer;
    double pos_x, pos_y, yaw, goal_x, goal_y, ps_val[8];
    bool initialized, goal_reached;
    State current_state; 
    WallType hugged_wall; 
    DirType travel_dir;
    double over_start_x, over_start_y, over_vx, over_vy, last_vx, last_vy;
    std::string last_status;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManhattanController>());
    rclcpp::shutdown();
    return 0;
}