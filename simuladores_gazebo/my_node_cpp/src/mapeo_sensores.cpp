#include <algorithm>
#include <cmath>
#include <geometry_msgs/msg/twist.hpp>
#include <iomanip>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sstream>
#include <std_msgs/msg/string.hpp>
#include <string>

using namespace std::placeholders;

#define ROBOT_SPEED 1.2
#define YAW_KP 2.5

#define WALL_FRONT_LIMIT 0.85
#define WALL_SIDE_LIMIT 0.85

#define NEW_WALL_DISTANCE_INF 0.01
#define NEW_WALL_DISTANCE_SUP 0.95
#define GOAL_MARGIN 0.50

#define MAX_X 16.5
#define MAX_Y 16.5
#define MIN_X 2.5
#define MIN_Y 2.5

#define COLOR_RESET "\033[0m"
#define COLOR_GREEN "\033[1;32m"
#define COLOR_YELLOW "\033[1;33m"
#define COLOR_RED "\033[1;31m"
#define COLOR_CYAN "\033[1;36m"
#define COLOR_BLUE "\033[1;34m"
#define COLOR_MAGENTA "\033[1;35m"
#define COLOR_WHITE "\033[1;37m"

enum State { LIBRE, SIGUIENDO_PARED, INT_CORNER, FINDING_WALL, META_ALCANZADA };
enum WallType { NONE, FRONT, BACK, LEFT, RIGHT };
enum DirType { STOP, FORWARD, BACKWARD, MOVE_LEFT, MOVE_RIGHT };

// Helpers de display
void printColored(const std::string &message,
                  const std::string &color = COLOR_RESET) {
  std::cout << color << message << COLOR_RESET << std::endl;
}

void clearScreen() { std::cout << "\033[2J\033[1;1H"; }

class ManhattanController : public rclcpp::Node {
public:
  ManhattanController() : Node("my_node_cpp") {
    auto qos = rclcpp::SensorDataQoS();
    pub_vel = this->create_publisher<geometry_msgs::msg::Twist>(
        "/model/my_diffdrive_robot/cmd_vel", 10);
    pub_state =
        this->create_publisher<std_msgs::msg::String>("/robot_status", 10);

    sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/model/my_diffdrive_robot/odometry", qos,
        [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
          update_state(msg->pose.pose.position.x, msg->pose.pose.position.y,
                       msg->pose.pose.orientation.z,
                       msg->pose.pose.orientation.w);
        });

    for (int i = 0; i < 8; ++i) {
      std::string topic = "/ps" + std::to_string(i);
      sub_ps[i] = this->create_subscription<sensor_msgs::msg::LaserScan>(
          topic, qos,
          [this, i](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
            if (!msg->ranges.empty())
              ps_val[i] =
                  *std::min_element(msg->ranges.begin(), msg->ranges.end());
          });
      ps_val[i] = 10.0;
    }

    timer = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&ManhattanController::control_loop, this));

    initialized = false;
    goal_reached = false;
    current_state = LIBRE;
    hugged_wall = NONE;
    next_hugged_wall = NONE;
    travel_dir = STOP;
    yaw = pos_x = pos_y = 0.0;
    start_x = start_y = goal_x = goal_y = 0.0;
    last_vx = last_vy = 0.0;
  }

private:
  void update_state(double x, double y, double qz, double qw) {
    pos_x = x;
    pos_y = y;
    yaw = 2.0 * std::atan2(qz, qw);

    if (!initialized && (std::abs(pos_x) > 0.1 || std::abs(pos_y) > 0.1)) {
      start_x = pos_x;
      start_y = pos_y;

      // Objetivo en esquina diagonalmente opuesta
      if (pos_x < 6.0 && pos_y < 6.0) {
        goal_x = MAX_X;
        goal_y = MAX_Y;
      } else if (pos_x > 6.0 && pos_y < 6.0) {
        goal_x = MIN_X;
        goal_y = MAX_Y;
      } else if (pos_x < 6.0 && pos_y > 6.0) {
        goal_x = MAX_X;
        goal_y = MIN_Y;
      } else {
        goal_x = MIN_X;
        goal_y = MIN_Y;
      }

      initialized = true;
    }
  }

  std::string color_for_dist(double dist) const {
    if (dist < 0.8)
      return COLOR_RED;
    if (dist < 1.5)
      return COLOR_YELLOW;
    return COLOR_GREEN;
  }

  std::string make_bar(double dist, double max_dist = 5.0) const {
    const int LEN = 20;
    // Tratar inf/nan como "lejos" (barra llena verde)
    bool is_inf = !std::isfinite(dist);
    double capped = is_inf ? max_dist : std::min(dist, max_dist);
    int filled = std::max(0, static_cast<int>((capped / max_dist) * LEN));
    std::string bar = "[";
    for (int i = 0; i < LEN; ++i)
      bar += (i < filled) ? "█" : "░";
    bar += "]";
    std::string color = is_inf ? COLOR_GREEN : color_for_dist(dist);
    return color + bar + COLOR_RESET;
  }

  std::string state_to_str(State s) const {
    switch (s) {
    case LIBRE:
      return "LIBRE";
    case SIGUIENDO_PARED:
      return "SIGUIENDO_PARED";
    case INT_CORNER:
      return "ESQUINA_INTERIOR";
    case FINDING_WALL:
      return "BUSCANDO_PARED";
    case META_ALCANZADA:
      return "META_ALCANZADA";
    default:
      return "DESCONOCIDO";
    }
  }
  std::string wall_to_str(WallType w) const {
    if (w == FRONT)
      return "FRONTAL";
    if (w == BACK)
      return "TRASERA";
    if (w == LEFT)
      return "IZQUIERDA";
    if (w == RIGHT)
      return "DERECHA";
    return "NINGUNA";
  }
  std::string dir_to_str(DirType d) const {
    if (d == FORWARD)
      return "ADELANTE";
    if (d == BACKWARD)
      return "ATRAS";
    if (d == MOVE_LEFT)
      return "IZQUIERDA";
    if (d == MOVE_RIGHT)
      return "DERECHA";
    return "PARADO";
  }

  void draw_dashboard() {
    double dist_to_goal =
        std::sqrt(std::pow(goal_x - pos_x, 2) + std::pow(goal_y - pos_y, 2));

    clearScreen();

    printColored("╔════════════════════════════════════════════════════════════"
                 "════════╗",
                 COLOR_CYAN);
    printColored("║             MANHATTAN CONTROLLER  —  ROS2 / Gazebo         "
                 "        ║",
                 COLOR_CYAN);
    printColored("╚════════════════════════════════════════════════════════════"
                 "════════╝",
                 COLOR_CYAN);
    std::cout << std::endl;

    // ── Estado ──
    printColored("ESTADO:", COLOR_YELLOW);
    std::cout << "  Estado actual : " << COLOR_CYAN
              << state_to_str(current_state) << COLOR_RESET << std::endl;
    std::cout << "  Pared seguida : " << COLOR_MAGENTA
              << wall_to_str(hugged_wall) << COLOR_RESET << std::endl;
    std::cout << "  Direccion     : " << COLOR_BLUE << dir_to_str(travel_dir)
              << COLOR_RESET << std::endl;
    std::cout << std::endl;

    // ── Posición / Objetivo ──
    printColored("POSICION:", COLOR_YELLOW);
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Inicio        : X=" << start_x << "  Y=" << start_y
              << std::endl;
    std::cout << "  Actual        : X=" << pos_x << "  Y=" << pos_y
              << std::endl;
    std::cout << "  Objetivo      : X=" << goal_x << "  Y=" << goal_y
              << std::endl;
    std::cout << "  Dist. a meta  : " << color_for_dist(dist_to_goal)
              << dist_to_goal << " m" << COLOR_RESET << std::endl;
    std::cout << std::endl;

    // ── Velocidad ──
    printColored("VELOCIDAD:", COLOR_YELLOW);
    std::cout << "  vX=" << std::setw(6) << last_vx << "  vY=" << std::setw(6)
              << last_vy << std::endl;
    std::cout << std::endl;

    // ── Sensores ──
    printColored("┌────────────────────────────────────────────────────────────"
                 "────────┐",
                 COLOR_BLUE);
    printColored("│  SENSORES EN TIEMPO REAL                                   "
                 "        │",
                 COLOR_BLUE);
    printColored("└────────────────────────────────────────────────────────────"
                 "────────┘",
                 COLOR_BLUE);

    auto print_sensor = [&](const std::string &name, double dist) {
      std::string dist_str;
      if (!std::isfinite(dist))
        dist_str = "> 5m";
      else {
        std::ostringstream oss;
        oss << std::fixed << std::setprecision(2) << dist << " m";
        dist_str = oss.str();
      }
      std::cout << "  " << std::left << std::setw(9) << name << ": "
                << std::setw(7) << dist_str << "  " << make_bar(dist)
                << std::endl;
    };

    print_sensor("FRONTAL (ps3,ps4)", std::min(ps_val[3], ps_val[4]));
    print_sensor("DIAG-IZQ (ps1)   ", ps_val[1]);
    print_sensor("LATERAL-D (ps2)  ", ps_val[2]);
    print_sensor("TRASERA (ps0,ps7)", std::min(ps_val[0], ps_val[7]));
    print_sensor("LATERAL-I (ps5)  ", ps_val[5]);
    print_sensor("DIAG-DER (ps6)   ", ps_val[6]);
    std::cout << std::endl;

    printColored(
        "────────────────────────────────────────────────────────────────────",
        COLOR_BLUE);
    std::cout << "  Presiona Ctrl+C para detener" << std::endl;
  }

  void control_loop() {
    if (!initialized || goal_reached)
      return;

    ++tick_count;
    double errX = goal_x - pos_x;
    double errY = goal_y - pos_y;

    // Condición de llegada
    if (std::abs(errX) < GOAL_MARGIN && std::abs(errY) < GOAL_MARGIN) {
      goal_reached = true;
      current_state = META_ALCANZADA;
      stop();
      draw_dashboard();
      printColored("★ OBJETIVO ALCANZADO ★", COLOR_GREEN);
      RCLCPP_INFO(this->get_logger(), "META ALCANZADA en X=%.2f Y=%.2f", pos_x,
                  pos_y);
      return;
    }

    bool wall_F =
        (ps_val[3] < WALL_FRONT_LIMIT || ps_val[4] < WALL_FRONT_LIMIT);
    bool wall_B =
        (ps_val[0] < WALL_FRONT_LIMIT || ps_val[7] < WALL_FRONT_LIMIT);
    bool wall_L = (ps_val[5] < WALL_SIDE_LIMIT);
    bool wall_R = (ps_val[2] < WALL_SIDE_LIMIT);
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = -YAW_KP * yaw;

    double vx_w = 0, vy_w = 0;

    switch (current_state) {
    case LIBRE: {
      bool xDone = (std::abs(errX) <= 0.3);
      bool yDone = (std::abs(errY) <= 0.3);
      if (!xDone && !yDone) {
        if (std::abs(errX) < std::abs(errY)) {
          vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
          travel_dir = (errX > 0) ? FORWARD : BACKWARD;
        } else {
          vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
          travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
        }
      } else if (!xDone) {
        vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
        travel_dir = (errX > 0) ? FORWARD : BACKWARD;
      } else if (!yDone) {
        vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
        travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
      }

      if (wall_F) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = FRONT;
        if (std::abs(vy_w) > 0)
          travel_dir = (vy_w > 0) ? MOVE_LEFT : MOVE_RIGHT;
        else
          travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
      } else if (wall_B) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = BACK;
        if (std::abs(vy_w) > 0)
          travel_dir = (vy_w > 0) ? MOVE_LEFT : MOVE_RIGHT;
        else
          travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
      } else if (wall_L) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = LEFT;
        if (std::abs(vx_w) > 0)
          travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        else
          travel_dir = (errX > 0) ? FORWARD : BACKWARD;
      } else if (wall_R) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = RIGHT;
        if (std::abs(vx_w) > 0)
          travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        else
          travel_dir = (errX > 0) ? FORWARD : BACKWARD;
      }
      break;
    }

    case SIGUIENDO_PARED: {
      // Conservar distancia a pared para calcular esquina
      if (hugged_wall == FRONT) {
        double d = std::min(ps_val[3], ps_val[4]);
        if (d < 1.0)
          last_wall_dist = d;
      } else if (hugged_wall == BACK) {
        double d = std::min(ps_val[0], ps_val[7]);
        if (d < 1.0)
          last_wall_dist = d;
      } else if (hugged_wall == LEFT) {
        if (ps_val[5] < 1.0)
          last_wall_dist = ps_val[5];
      } else if (hugged_wall == RIGHT) {
        if (ps_val[2] < 1.0)
          last_wall_dist = ps_val[2];
      }

      // Esquina interior
      if ((wall_F && wall_L) || (wall_F && wall_R) || (wall_B && wall_L) ||
          (wall_B && wall_R)) {
        prev_hugged_wall = hugged_wall;
        current_state = INT_CORNER;
        break;
      }

      // Esquina exterior
      double turn_dist_target;
      if (hugged_wall == BACK ||
          ((hugged_wall == LEFT || hugged_wall == RIGHT) && last_vx < 0)) {
        turn_dist_target = NEW_WALL_DISTANCE_SUP;
      } else {
        turn_dist_target = NEW_WALL_DISTANCE_INF;
      }
      double turn_threshold = std::sqrt(last_wall_dist * last_wall_dist +
                                        turn_dist_target * turn_dist_target);

      bool lost_F = (hugged_wall == FRONT && ps_val[3] >= turn_threshold &&
                     ps_val[4] >= turn_threshold);
      bool lost_B = (hugged_wall == BACK && ps_val[0] >= turn_threshold &&
                     ps_val[7] >= turn_threshold);
      bool lost_L = (hugged_wall == LEFT && ps_val[5] >= turn_threshold &&
                     ps_val[1] >= turn_threshold);
      bool lost_R = (hugged_wall == RIGHT && ps_val[2] >= turn_threshold &&
                     ps_val[6] >= turn_threshold);

      if (lost_F || lost_B || lost_L || lost_R) {
        corner_start_x = pos_x;
        corner_start_y = pos_y;
        if (lost_B || (lost_L && last_vx < 0) || (lost_R && last_vx < 0)) {
          corner_travel_target = NEW_WALL_DISTANCE_SUP;
        } else {
          // Esquinas inferiores del mapa
          corner_travel_target = NEW_WALL_DISTANCE_INF;
        }

        corner_phase = 0;
        corner_pre_vx = last_vx;
        corner_pre_vy = last_vy;

        double turn_vx = 0.0, turn_vy = 0.0;
        WallType new_hugged = hugged_wall;
        DirType new_dir = travel_dir;

        if (lost_B) {
          if (last_vy > 0) {
            turn_vx = ROBOT_SPEED;
            turn_vy = 0;
            new_hugged = LEFT;
            new_dir = FORWARD;
          } else {
            turn_vx = ROBOT_SPEED;
            turn_vy = 0;
            new_hugged = RIGHT;
            new_dir = FORWARD;
          }
        } else if (lost_F) {
          if (last_vy > 0) {
            turn_vx = -ROBOT_SPEED;
            turn_vy = 0;
            new_hugged = LEFT;
            new_dir = BACKWARD;
          } else {
            turn_vx = -ROBOT_SPEED;
            turn_vy = 0;
            new_hugged = RIGHT;
            new_dir = BACKWARD;
          }
        } else if (lost_L) {
          if (last_vx > 0) {
            turn_vx = 0;
            turn_vy = +ROBOT_SPEED;
            new_hugged = RIGHT;
            new_dir = MOVE_RIGHT;
          } else {
            turn_vx = 0;
            turn_vy = -ROBOT_SPEED;
            new_hugged = LEFT;
            new_dir = MOVE_LEFT;
          }
        } else if (lost_R) {
          if (last_vx > 0) {
            turn_vx = 0;
            turn_vy = -ROBOT_SPEED;
            new_hugged = LEFT;
            new_dir = MOVE_LEFT;
          } else {
            turn_vx = 0;
            turn_vy = -ROBOT_SPEED;
            new_hugged = FRONT;
            new_dir = MOVE_LEFT;
          }
        }

        corner_turn_vx = turn_vx;
        corner_turn_vy = turn_vy;
        corner_turn_dir = new_dir;
        next_hugged_wall = new_hugged;

        vx_w = corner_pre_vx;
        vy_w = corner_pre_vy;

        if (std::abs(vx_w) >= std::abs(vy_w)) {
          if (vx_w > 0)
            travel_dir = BACKWARD;
          else if (vx_w < 0)
            travel_dir = FORWARD;
          else
            travel_dir = STOP;
        } else {
          if (vy_w > 0)
            travel_dir = MOVE_RIGHT;
          else if (vy_w < 0)
            travel_dir = MOVE_LEFT;
          else
            travel_dir = STOP;
        }

        current_state = FINDING_WALL;
        break;
      }

      // Seguimiento normal
      if (hugged_wall == FRONT || hugged_wall == BACK) {
        vx_w = 0.0;
        if (travel_dir == MOVE_LEFT) {
          vy_w = ROBOT_SPEED;
        } else if (travel_dir == MOVE_RIGHT) {
          vy_w = -ROBOT_SPEED;
        } else {
          vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
          travel_dir = (vy_w > 0) ? MOVE_LEFT : MOVE_RIGHT;
        }
      } else if (hugged_wall == LEFT || hugged_wall == RIGHT) {
        vy_w = 0.0;
        if (travel_dir == FORWARD) {
          vx_w = ROBOT_SPEED;
        } else if (travel_dir == BACKWARD) {
          vx_w = -ROBOT_SPEED;
        } else {
          vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
          travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        }
      }
      break;
    }

    // ── ESQUINA INTERIOR ───────────────────────────────────────────
    case INT_CORNER:
      if (wall_F && wall_L) { // 1 Sup izqda
        if (prev_hugged_wall == FRONT) {
          vx_w = ROBOT_SPEED;
          vy_w = 0;
          travel_dir = FORWARD;
          hugged_wall = LEFT;
        } else {
          vx_w = 0;
          vy_w = ROBOT_SPEED;
          travel_dir = MOVE_LEFT;
          hugged_wall = FRONT;
        }
      } else if (wall_B && wall_R) { // 4 Inf drcha
        if (prev_hugged_wall == BACK) {
          vx_w = -ROBOT_SPEED;
          vy_w = 0;
          travel_dir = BACKWARD;
          hugged_wall = RIGHT;
        } else {
          vx_w = 0;
          vy_w = -ROBOT_SPEED;
          travel_dir = MOVE_RIGHT;
          hugged_wall = BACK;
        }
      } else if (wall_F && wall_R) { // 2 Sup drcha
        if (prev_hugged_wall == FRONT) {
          vx_w = ROBOT_SPEED;
          vy_w = 0;
          travel_dir = FORWARD;
          hugged_wall = RIGHT;
        } else {
          vx_w = 0;
          vy_w = -ROBOT_SPEED;
          travel_dir = MOVE_RIGHT;
          hugged_wall = FRONT;
        }
      } else if (wall_B && wall_L) { // 3 Inf izqda
        if (prev_hugged_wall == BACK) {
          vx_w = -ROBOT_SPEED;
          vy_w = 0;
          travel_dir = BACKWARD;
          hugged_wall = LEFT;
        } else {
          vx_w = 0;
          vy_w = ROBOT_SPEED;
          travel_dir = MOVE_LEFT;
          hugged_wall = BACK;
        }
      }

      // Salida del corner cuando dejamos de ver alguna de las paredes
      // interiores
      if (!((wall_F && wall_L) || (wall_F && wall_R) || (wall_B && wall_L) ||
            (wall_B && wall_R))) {
        current_state = SIGUIENDO_PARED;
      }
      break;

    case FINDING_WALL: {
      // Fase 0: avanzar recto, Fase 1: rodear la esquina
      if (corner_phase == 0) {
        vx_w = corner_pre_vx;
        vy_w = corner_pre_vy;

        double dist_from_corner =
            std::sqrt(std::pow(pos_x - corner_start_x, 2) +
                      std::pow(pos_y - corner_start_y, 2));
        if (dist_from_corner >= corner_travel_target) {
          corner_phase = 1;
        }
      } else {
        // Fase 1: rodear la esquina con la velocidad calculada
        vx_w = corner_turn_vx;
        vy_w = corner_turn_vy;
        travel_dir = corner_turn_dir;
      }

      bool see_wall = false;
      double check_limit = WALL_SIDE_LIMIT + 0.50;
      if (next_hugged_wall == FRONT)
        see_wall = (ps_val[3] < check_limit || ps_val[4] < check_limit);
      else if (next_hugged_wall == BACK)
        see_wall = (ps_val[0] < check_limit || ps_val[7] < check_limit);
      else if (next_hugged_wall == LEFT)
        see_wall = (ps_val[5] < check_limit);
      else if (next_hugged_wall == RIGHT)
        see_wall = (ps_val[2] < check_limit);

      bool obstacle = false;
      if (travel_dir == FORWARD && wall_F)
        obstacle = true;
      if (travel_dir == BACKWARD && wall_B)
        obstacle = true;
      if (travel_dir == MOVE_LEFT && wall_L)
        obstacle = true;
      if (travel_dir == MOVE_RIGHT && wall_R)
        obstacle = true;

      if (see_wall || obstacle) {
        if (next_hugged_wall != NONE) {
          hugged_wall = next_hugged_wall;
        }
        current_state = SIGUIENDO_PARED;
      }
      break;
    }

    case META_ALCANZADA:
      return;
    }

    last_vx = vx_w;
    last_vy = vy_w;

    // Transformar velocidades mundo -> robot
    twist.linear.x = vx_w * std::cos(yaw) + vy_w * std::sin(yaw);
    twist.linear.y = -vx_w * std::sin(yaw) + vy_w * std::cos(yaw);
    pub_vel->publish(twist);

    if (tick_count % 5 == 0)
      draw_dashboard();
  }

  void stop() { pub_vel->publish(geometry_msgs::msg::Twist()); }
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_vel;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_state;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_ps[8];
  rclcpp::TimerBase::SharedPtr timer;

  double pos_x, pos_y, yaw;
  double goal_x, goal_y, start_x, start_y;
  double ps_val[8];
  bool initialized, goal_reached;

  State current_state;
  WallType hugged_wall;
  WallType next_hugged_wall;
  WallType prev_hugged_wall;
  DirType travel_dir;

  double last_vx = 0.0, last_vy = 0.0;
  int tick_count = 0; // Contador de ticks para throttle del dashboard
  double last_wall_dist = 0.75;
  double corner_start_x = 0.0, corner_start_y = 0.0;
  double corner_travel_target = 0.0;
  double corner_pre_vx = 0.0, corner_pre_vy = 0.0;
  double corner_turn_vx = 0.0, corner_turn_vy = 0.0;
  DirType corner_turn_dir = STOP;
  int corner_phase = 0;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManhattanController>());
  rclcpp::shutdown();
  return 0;
}