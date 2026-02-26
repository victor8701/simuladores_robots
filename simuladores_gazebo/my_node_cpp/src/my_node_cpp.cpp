#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <string>
#include <algorithm>
#include <sstream>

using namespace std::placeholders;

// ==================== VELOCIDADES ====================
#define ROBOT_SPEED      1.2
#define YAW_KP           2.5

// ==================== UMBRALES DE SENSORES ====================
#define WALL_FRONT_LIMIT 0.75
#define WALL_SIDE_LIMIT  0.75

// ==================== DISTANCIAS ====================
#define OVERSHOOT_DIST   0.85    // Metros que avanza tras esquina exterior
#define GOAL_MARGIN      0.50    // Margen de llegada al objetivo (metros)

// ==================== LIMITES DEL MAPA ====================
#define MAX_X 16.5
#define MAX_Y 16.5
#define MIN_X  2.5
#define MIN_Y  2.5

// ==================== DISPLAY ====================
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[1;32m"
#define COLOR_YELLOW  "\033[1;33m"
#define COLOR_RED     "\033[1;31m"
#define COLOR_CYAN    "\033[1;36m"
#define COLOR_BLUE    "\033[1;34m"
#define COLOR_MAGENTA "\033[1;35m"
#define COLOR_WHITE   "\033[1;37m"

enum State     { LIBRE, SIGUIENDO_PARED, INT_CORNER, OVERSHOOT, META_ALCANZADA };
enum WallType  { NONE, FRONT, BACK, LEFT, RIGHT };
enum DirType   { STOP, FORWARD, BACKWARD, MOVE_LEFT, MOVE_RIGHT };

// ---------------------------------------------------------------------------
// Helpers de display (réplica de test_sensores.cpp / e-puck_avoid_obstacles_VMP.cpp)
// ---------------------------------------------------------------------------
void printColored(const std::string& message, const std::string& color = COLOR_RESET) {
    std::cout << color << message << COLOR_RESET << std::endl;
}

void clearScreen() {
    std::cout << "\033[2J\033[1;1H";
}

// ---------------------------------------------------------------------------

class ManhattanController : public rclcpp::Node {
public:
    ManhattanController() : Node("my_node_cpp") {
        auto qos = rclcpp::SensorDataQoS();
        pub_vel   = this->create_publisher<geometry_msgs::msg::Twist>("/model/my_diffdrive_robot/cmd_vel", 10);
        pub_state = this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

        sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/my_diffdrive_robot/odometry", qos,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                update_state(
                    msg->pose.pose.position.x,
                    msg->pose.pose.position.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
            });

        for (int i = 0; i < 8; ++i) {
            std::string topic = "/ps" + std::to_string(i);
            sub_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
                topic, qos, [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                    if (!msg->ranges.empty())
                        ps_val[i] = *std::min_element(msg->ranges.begin(), msg->ranges.end());
                });
            ps_val[i] = 10.0;
        }

        timer = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ManhattanController::control_loop, this));

        initialized   = false;
        goal_reached  = false;
        current_state = LIBRE;
        hugged_wall   = NONE;
        travel_dir    = STOP;
        yaw = pos_x = pos_y = 0.0;
        start_x = start_y = goal_x = goal_y = 0.0;
        over_start_x = over_start_y = over_vx = over_vy = 0.0;
        last_vx = last_vy = 0.0;
    }

private:
    // -----------------------------------------------------------------------
    // Odometría
    // -----------------------------------------------------------------------
    void update_state(double x, double y, double qz, double qw) {
        pos_x = x;  pos_y = y;
        yaw   = 2.0 * std::atan2(qz, qw);

        if (!initialized && (std::abs(pos_x) > 0.1 || std::abs(pos_y) > 0.1)) {
            start_x = pos_x;
            start_y = pos_y;

            // Esquina diagonalmente opuesta según quadrante de inicio
            if      (pos_x < 6.0 && pos_y < 6.0) { goal_x = MAX_X; goal_y = MAX_Y; }
            else if (pos_x > 6.0 && pos_y < 6.0) { goal_x = MIN_X; goal_y = MAX_Y; }
            else if (pos_x < 6.0 && pos_y > 6.0) { goal_x = MAX_X; goal_y = MIN_Y; }
            else                                  { goal_x = MIN_X; goal_y = MIN_Y; }

            initialized = true;
        }
    }

    // -----------------------------------------------------------------------
    // Helpers de display
    // -----------------------------------------------------------------------
    std::string color_for_dist(double dist) const {
        if (dist < 0.8)  return COLOR_RED;
        if (dist < 1.5)  return COLOR_YELLOW;
        return COLOR_GREEN;
    }

    std::string make_bar(double dist, double max_dist = 5.0) const {
        const int LEN  = 20;
        // Tratar inf/nan como "lejos" (barra llena verde)
        bool is_inf = !std::isfinite(dist);
        double capped = is_inf ? max_dist : std::min(dist, max_dist);
        int filled = std::max(0, static_cast<int>((capped / max_dist) * LEN));
        std::string bar = "[";
        for (int i = 0; i < LEN; ++i) bar += (i < filled) ? "█" : "░";
        bar += "]";
        // inf = verde (no hay obstáculo)
        std::string color = is_inf ? COLOR_GREEN : color_for_dist(dist);
        return color + bar + COLOR_RESET;
    }

    std::string state_to_str(State s) const {
        switch(s) {
            case LIBRE:           return "LIBRE";
            case SIGUIENDO_PARED: return "SIGUIENDO_PARED";
            case INT_CORNER:      return "ESQUINA_INTERIOR";
            case OVERSHOOT:       return "OVERSHOOT";
            case META_ALCANZADA:  return "META_ALCANZADA";
            default:              return "DESCONOCIDO";
        }
    }
    std::string wall_to_str(WallType w) const {
        if (w == FRONT) return "FRONTAL";
        if (w == BACK)  return "TRASERA";
        if (w == LEFT)  return "IZQUIERDA";
        if (w == RIGHT) return "DERECHA";
        return "NINGUNA";
    }
    std::string dir_to_str(DirType d) const {
        if (d == FORWARD)    return "ADELANTE";
        if (d == BACKWARD)   return "ATRAS";
        if (d == MOVE_LEFT)  return "IZQUIERDA";
        if (d == MOVE_RIGHT) return "DERECHA";
        return "PARADO";
    }

    // -----------------------------------------------------------------------
    // Dashboard — estilo test_sensores.cpp
    // -----------------------------------------------------------------------
    void draw_dashboard() {
        double dist_to_goal = std::sqrt(std::pow(goal_x - pos_x, 2) + std::pow(goal_y - pos_y, 2));

        clearScreen();

        // ── Cabecera ──
        printColored("╔════════════════════════════════════════════════════════════════════╗", COLOR_CYAN);
        printColored("║             MANHATTAN CONTROLLER  —  ROS2 / Gazebo                ║", COLOR_CYAN);
        printColored("╚════════════════════════════════════════════════════════════════════╝", COLOR_CYAN);
        std::cout << std::endl;

        // ── Estado ──
        printColored("ESTADO:", COLOR_YELLOW);
        std::cout << "  Estado actual : " << COLOR_CYAN  << state_to_str(current_state) << COLOR_RESET << std::endl;
        std::cout << "  Pared seguida : " << COLOR_MAGENTA << wall_to_str(hugged_wall)  << COLOR_RESET << std::endl;
        std::cout << "  Direccion     : " << COLOR_BLUE  << dir_to_str(travel_dir)      << COLOR_RESET << std::endl;
        std::cout << std::endl;

        // ── Posición / Objetivo ──
        printColored("POSICION:", COLOR_YELLOW);
        std::cout << std::fixed << std::setprecision(2);
        std::cout << "  Inicio        : X=" << start_x << "  Y=" << start_y << std::endl;
        std::cout << "  Actual        : X=" << pos_x   << "  Y=" << pos_y   << std::endl;
        std::cout << "  Objetivo      : X=" << goal_x  << "  Y=" << goal_y  << std::endl;
        std::cout << "  Dist. a meta  : " << color_for_dist(dist_to_goal)
                  << dist_to_goal << " m" << COLOR_RESET << std::endl;
        std::cout << std::endl;

        // ── Velocidad ──
        printColored("VELOCIDAD:", COLOR_YELLOW);
        std::cout << "  vX=" << std::setw(6) << last_vx
                  << "  vY=" << std::setw(6) << last_vy << std::endl;
        std::cout << std::endl;

        // ── Sensores ──
        printColored("┌────────────────────────────────────────────────────────────────────┐", COLOR_BLUE);
        printColored("│  SENSORES EN TIEMPO REAL                                           │", COLOR_BLUE);
        printColored("└────────────────────────────────────────────────────────────────────┘", COLOR_BLUE);

        auto print_sensor = [&](const std::string& name, double dist) {
            std::string dist_str;
            if (!std::isfinite(dist)) dist_str = "> 5m";
            else { std::ostringstream oss; oss << std::fixed << std::setprecision(2) << dist << " m"; dist_str = oss.str(); }
            std::cout << "  " << std::left << std::setw(9) << name
                      << ": " << std::setw(7) << dist_str << "  " << make_bar(dist) << std::endl;
        };

        print_sensor("FRONTAL",  std::min(ps_val[0], ps_val[7]));
        print_sensor("DIAG-IZQ", ps_val[1]);
        print_sensor("LATERAL-D",ps_val[2]);
        print_sensor("TRASERA",  std::min(ps_val[3], ps_val[4]));
        print_sensor("LATERAL-I",ps_val[5]);
        print_sensor("DIAG-DER", ps_val[6]);
        std::cout << std::endl;

        printColored("────────────────────────────────────────────────────────────────────", COLOR_BLUE);
        std::cout << "  Presiona Ctrl+C para detener" << std::endl;
    }

    // -----------------------------------------------------------------------
    // Bucle de control (100 ms) — dashboard se actualiza cada 5 ticks (~500ms)
    // -----------------------------------------------------------------------
    void control_loop() {
        if (!initialized || goal_reached) return;

        ++tick_count;
        double errX = goal_x - pos_x;
        double errY = goal_y - pos_y;

        // ── Condición de llegada ──
        //if (std::sqrt(errX*errX + errY*errY) < GOAL_RADIUS) {
        if (std::abs(errX) < GOAL_MARGIN && std::abs(errY) < GOAL_MARGIN) {
            goal_reached  = true;
            current_state = META_ALCANZADA;
            stop();
            draw_dashboard();
            printColored("★ OBJETIVO ALCANZADO ★", COLOR_GREEN);
            std::cout << "  Posicion final: X=" << std::fixed << std::setprecision(2)
                      << pos_x << "  Y=" << pos_y << std::endl;
            RCLCPP_INFO(this->get_logger(), "META ALCANZADA en X=%.2f Y=%.2f", pos_x, pos_y);
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
            // ── LIBRE ──────────────────────────────────────────────────────
            case LIBRE: {
                // Eje con MENOR distancia al objetivo (no ir en diagonal)
                // Bug fix: si un eje ya está dentro del umbral, moverse por el otro
                bool xDone = (std::abs(errX) <= 0.3);
                bool yDone = (std::abs(errY) <= 0.3);

                if (!xDone && !yDone) {
                    // Ambos ejes pendientes: primero el de MENOR error
                    if (std::abs(errX) < std::abs(errY)) {
                        vx_w       = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errX > 0) ? FORWARD : BACKWARD;
                    } else {
                        vy_w       = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
                    }
                } else if (!xDone) {
                    // Y ya alcanzado, mover en X
                    vx_w       = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                    travel_dir = (errX > 0) ? FORWARD : BACKWARD;
                } else if (!yDone) {
                    // X ya alcanzado, mover en Y
                    vy_w       = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                    travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
                }
                // Si ambos done -> GOAL_MARGIN lo capturará en el siguiente tick

                if      (wall_F) { current_state = SIGUIENDO_PARED; hugged_wall = FRONT; }
                else if (wall_B) { current_state = SIGUIENDO_PARED; hugged_wall = BACK;  }
                else if (wall_L) { current_state = SIGUIENDO_PARED; hugged_wall = LEFT;  }
                else if (wall_R) { current_state = SIGUIENDO_PARED; hugged_wall = RIGHT; }
                break;
            }

            // ── SIGUIENDO_PARED ────────────────────────────────────────────
            case SIGUIENDO_PARED:
                // Esquina interior
                if ((wall_F && wall_L) || (wall_F && wall_R) ||
                    (wall_B && wall_L) || (wall_B && wall_R)) {
                    prev_hugged_wall = hugged_wall;
                    current_state    = INT_CORNER;
                    break;
                }

                // Esquina exterior: la pared seguida desaparece
                if ((hugged_wall == FRONT && !wall_F && ps_val[0] > 1.2 && ps_val[7] > 1.2) ||
                    (hugged_wall == BACK  && !wall_B && ps_val[3] > 1.2 && ps_val[4] > 1.2) ||
                    (hugged_wall == LEFT  && !wall_L && ps_val[2] > 1.2) ||
                    (hugged_wall == RIGHT && !wall_R && ps_val[5] > 1.2)) {
                    start_overshoot(last_vx, last_vy);
                    break;
                }

                // Seguimiento normal: bloquear eje de la pared, moverse en el perpendicular
                if (hugged_wall == FRONT || hugged_wall == BACK) {
                    if (std::abs(errY) > 0.3) {
                        vy_w       = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
                    }
                } else {
                    if (std::abs(errX) > 0.3) {
                        vx_w       = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        travel_dir = (errX > 0) ? FORWARD : BACKWARD;
                    }
                }
                break;

            // ── ESQUINA INTERIOR ───────────────────────────────────────────
            case INT_CORNER:
                // Escape según memoria de la pared de procedencia
                if (wall_F && wall_L) {                         // SUP-IZQ
                    if (prev_hugged_wall == LEFT)
                        { vx_w = 0; vy_w = -ROBOT_SPEED; travel_dir = MOVE_RIGHT; }
                    else
                        { vx_w = -ROBOT_SPEED; vy_w = 0; travel_dir = BACKWARD; }
                } else if (wall_F && wall_R) {                  // SUP-DER
                    if (prev_hugged_wall == RIGHT)
                        { vx_w = 0; vy_w = ROBOT_SPEED; travel_dir = MOVE_LEFT; }
                    else
                        { vx_w = -ROBOT_SPEED; vy_w = 0; travel_dir = BACKWARD; }
                } else if (wall_B && wall_L) {                  // INF-IZQ
                    if (prev_hugged_wall == LEFT)
                        { vx_w = 0; vy_w = -ROBOT_SPEED; travel_dir = MOVE_RIGHT; }
                    else
                        { vx_w = ROBOT_SPEED; vy_w = 0; travel_dir = FORWARD; }
                } else if (wall_B && wall_R) {                  // INF-DER
                    if (prev_hugged_wall == RIGHT)
                        { vx_w = 0; vy_w = ROBOT_SPEED; travel_dir = MOVE_LEFT; }
                    else
                        { vx_w = ROBOT_SPEED; vy_w = 0; travel_dir = FORWARD; }
                }

                // Actualizar pared seguida según dirección de movimiento
                if      (std::abs(vx_w) > 0) { hugged_wall = wall_L ? LEFT  : (wall_R ? RIGHT : hugged_wall); }
                else if (std::abs(vy_w) > 0) { hugged_wall = wall_F ? FRONT : (wall_B ? BACK  : hugged_wall); }

                // Salida del corner cuando no hay ninguna pared
                if (!wall_F && !wall_B && !wall_L && !wall_R) {
                    current_state = LIBRE;
                    hugged_wall   = NONE;
                }
                break;

            // ── OVERSHOOT (esquina exterior) ───────────────────────────────
            case OVERSHOOT: {
                double d = std::sqrt(std::pow(pos_x - over_start_x, 2) +
                                     std::pow(pos_y - over_start_y, 2));
                if (d >= OVERSHOOT_DIST) {
                    // Giro contextual: bloquear eje actual, liberar perpendicular
                    if (hugged_wall == FRONT) {
                        vx_w = (last_vy < 0) ? -ROBOT_SPEED : ROBOT_SPEED;
                        vy_w = 0;
                        hugged_wall = (last_vy < 0) ? RIGHT : LEFT;
                        travel_dir  = (last_vy < 0) ? BACKWARD : FORWARD;
                    } else if (hugged_wall == BACK) {
                        vx_w = (last_vy < 0) ? -ROBOT_SPEED : ROBOT_SPEED;
                        vy_w = 0;
                        hugged_wall = (last_vy < 0) ? LEFT : RIGHT;
                        travel_dir  = (last_vy < 0) ? BACKWARD : FORWARD;
                    } else if (hugged_wall == LEFT) {
                        vx_w = 0;
                        vy_w = (last_vx > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
                        hugged_wall = (last_vx > 0) ? BACK : FRONT;
                        travel_dir  = (last_vx > 0) ? MOVE_LEFT : MOVE_RIGHT;
                    } else if (hugged_wall == RIGHT) {
                        vx_w = 0;
                        vy_w = (last_vx > 0) ? -ROBOT_SPEED : ROBOT_SPEED;
                        hugged_wall = (last_vx > 0) ? FRONT : BACK;
                        travel_dir  = (last_vx > 0) ? MOVE_RIGHT : MOVE_LEFT;
                    }
                    current_state = SIGUIENDO_PARED;
                } else {
                    vx_w = over_vx;
                    vy_w = over_vy;
                }
                break;
            }

            case META_ALCANZADA:
                return;
        }

        last_vx = vx_w;
        last_vy = vy_w;

        // Transformar velocidades mundo → robot (rotación por yaw)
        twist.linear.x =  vx_w * std::cos(yaw) + vy_w * std::sin(yaw);
        twist.linear.y = -vx_w * std::sin(yaw) + vy_w * std::cos(yaw);
        pub_vel->publish(twist);

        // Actualizar display cada 5 ticks (~500ms), como test_sensores.cpp con frameCount%10
        if (tick_count % 5 == 0) draw_dashboard();
    }

    // -----------------------------------------------------------------------
    // Auxiliares
    // -----------------------------------------------------------------------
    void start_overshoot(double vx, double vy) {
        current_state = OVERSHOOT;
        over_start_x  = pos_x;
        over_start_y  = pos_y;
        over_vx = vx;
        over_vy = vy;
    }

    void stop() {
        pub_vel->publish(geometry_msgs::msg::Twist());
    }

    // -----------------------------------------------------------------------
    // Miembros
    // -----------------------------------------------------------------------
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr     pub_state;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ps[8];
    rclcpp::TimerBase::SharedPtr timer;

    double pos_x, pos_y, yaw;
    double goal_x, goal_y, start_x, start_y;
    double ps_val[8];
    bool   initialized, goal_reached;

    State    current_state;
    WallType hugged_wall;
    WallType prev_hugged_wall;
    DirType  travel_dir;

    double over_start_x, over_start_y, over_vx, over_vy;
    double last_vx, last_vy;
    int    tick_count = 0;   // Contador de ticks para throttle del dashboard
};

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManhattanController>());
    rclcpp::shutdown();
    return 0;
}