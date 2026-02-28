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

// ==================== VELOCIDADES ====================
#define ROBOT_SPEED 1.2
#define YAW_KP 2.5

// ==================== UMBRALES DE SENSORES ====================
#define WALL_FRONT_LIMIT 0.85
#define WALL_SIDE_LIMIT 0.85

// ==================== DISTANCIAS ====================
// Distancia (m) que se avanza tras una esquina exterior
// antes de volver a buscar la siguiente pared
#define NEW_WALL_DISTANCE_INF 0.01 // Esquinas inferiores
#define NEW_WALL_DISTANCE_SUP 0.95 // Esquinas superiores
#define GOAL_MARGIN 0.50           // Margen de llegada al objetivo (metros)

// ==================== LIMITES DEL MAPA ====================
#define MAX_X 16.5
#define MAX_Y 16.5
#define MIN_X 2.5
#define MIN_Y 2.5

// ==================== DISPLAY ====================
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

// ---------------------------------------------------------------------------
// Helpers de display (réplica de test_sensores.cpp /
// e-puck_avoid_obstacles_VMP.cpp)
// ---------------------------------------------------------------------------
void printColored(const std::string &message,
                  const std::string &color = COLOR_RESET) {
  std::cout << color << message << COLOR_RESET << std::endl;
}

void clearScreen() { std::cout << "\033[2J\033[1;1H"; }

// ---------------------------------------------------------------------------

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
  // -----------------------------------------------------------------------
  // Odometría
  // -----------------------------------------------------------------------
  void update_state(double x, double y, double qz, double qw) {
    pos_x = x;
    pos_y = y;
    yaw = 2.0 * std::atan2(qz, qw);

    if (!initialized && (std::abs(pos_x) > 0.1 || std::abs(pos_y) > 0.1)) {
      start_x = pos_x;
      start_y = pos_y;

      // Esquina diagonalmente opuesta según quadrante de inicio
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

  // -----------------------------------------------------------------------
  // Helpers de display
  // -----------------------------------------------------------------------
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
    // inf = verde (no hay obstáculo)
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

  // -----------------------------------------------------------------------
  // Dashboard — estilo test_sensores.cpp
  // -----------------------------------------------------------------------
  void draw_dashboard() {
    double dist_to_goal =
        std::sqrt(std::pow(goal_x - pos_x, 2) + std::pow(goal_y - pos_y, 2));

    clearScreen();

    // ── Cabecera ──
    printColored("╔════════════════════════════════════════════════════════════════════╗", COLOR_CYAN);
    printColored("║             MANHATTAN CONTROLLER  —  ROS2 / Gazebo                 ║", COLOR_CYAN);
    printColored("╚════════════════════════════════════════════════════════════════════╝", COLOR_CYAN);
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

  // -----------------------------------------------------------------------
  // Bucle de control (100 ms) — dashboard se actualiza cada 5 ticks (~500ms)
  // -----------------------------------------------------------------------
  void control_loop() {
    if (!initialized || goal_reached)
      return;

    ++tick_count;
    double errX = goal_x - pos_x;
    double errY = goal_y - pos_y;

    // ── Condición de llegada ──
    // if (std::sqrt(errX*errX + errY*errY) < GOAL_RADIUS) {
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

    bool wall_F = (ps_val[3] < WALL_FRONT_LIMIT || ps_val[4] < WALL_FRONT_LIMIT);
    bool wall_B = (ps_val[0] < WALL_FRONT_LIMIT || ps_val[7] < WALL_FRONT_LIMIT);
    // Para decidir si hay pared lateral (y para entrar en esquina interior),
    // usamos únicamente los sensores laterales puros (ps5 izquierda, ps2 derecha).
    // Los diagonales (ps1, ps6) pueden ver esquinas/techos y no deben
    // provocar un cambio de estado a ESQUINA_INTERIOR.
    bool wall_L = (ps_val[5] < WALL_SIDE_LIMIT);
    bool wall_R = (ps_val[2] < WALL_SIDE_LIMIT);
    auto twist = geometry_msgs::msg::Twist();
    twist.angular.z = -YAW_KP * yaw;

    double vx_w = 0, vy_w = 0;

    switch (current_state) {
    // ── LIBRE ──────────────────────────────────────────────────────
    case LIBRE: {
      bool xDone = (std::abs(errX) <= 0.3);
      bool yDone = (std::abs(errY) <= 0.3);
      if (!xDone && !yDone) {
        if (std::abs(errX) < std::abs(errY)) {
          vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED; 
          travel_dir = (errX > 0) ? FORWARD : BACKWARD; // +X=Alante(Inf), -X=Atras(Sup)
        } else {
          vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED; 
          travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT; // +Y=Izquierda(Drcha Mapa), -Y=Derecha(Izq Mapa)
        }
      } else if (!xDone) {
        vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
        travel_dir = (errX > 0) ? FORWARD : BACKWARD;
      } else if (!yDone) {
        vy_w = (errY > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
        travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT;
      }
      // Si ambos done -> GOAL_MARGIN lo capturará en el siguiente tick

      if (wall_F) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = FRONT;
        // Keep the parallel velocity that brought us here if moving perpendicular to the wall.
        // Or if moving towards the wall, pick the direction with the largest error.
        if (std::abs(vy_w) > 0) travel_dir = (vy_w > 0) ? MOVE_LEFT : MOVE_RIGHT;
        else travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT; 
      } else if (wall_B) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = BACK;
        if (std::abs(vy_w) > 0) travel_dir = (vy_w > 0) ? MOVE_LEFT : MOVE_RIGHT;
        else travel_dir = (errY > 0) ? MOVE_LEFT : MOVE_RIGHT; 
      } else if (wall_L) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = LEFT;
        if (std::abs(vx_w) > 0) travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        else travel_dir = (errX > 0) ? FORWARD : BACKWARD; 
      } else if (wall_R) {
        current_state = SIGUIENDO_PARED;
        hugged_wall = RIGHT;
        if (std::abs(vx_w) > 0) travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        else travel_dir = (errX > 0) ? FORWARD : BACKWARD; 
      }
      break;
    }

    // ── SIGUIENDO_PARED ────────────────────────────────────────────
    case SIGUIENDO_PARED: {
      // Conservamos la distancia a la pared para calcular la hipotenusa de la esquina
      if (hugged_wall == FRONT) {
        double d = std::min(ps_val[3], ps_val[4]);
        if (d < 1.0) last_wall_dist = d;
      } else if (hugged_wall == BACK) {
        double d = std::min(ps_val[0], ps_val[7]);
        if (d < 1.0) last_wall_dist = d;
      } else if (hugged_wall == LEFT) {
        if (ps_val[5] < 1.0) last_wall_dist = ps_val[5]; // LATERAL-I
      } else if (hugged_wall == RIGHT) {
        if (ps_val[2] < 1.0) last_wall_dist = ps_val[2]; // LATERAL-D
      }

      // Esquina interior
      if ((wall_F && wall_L) || (wall_F && wall_R) || (wall_B && wall_L) ||
          (wall_B && wall_R)) {
        prev_hugged_wall = hugged_wall;
        current_state = INT_CORNER;
        break;
      }

      // Esquina exterior: calculamos a qué distancia del sensor (hipotenusa) significa
      // que ya hemos superado la esquina exactamente por la longitud NEW_WALL_DISTANCE
      double turn_dist_target;
      if (hugged_wall == BACK || ((hugged_wall == LEFT || hugged_wall == RIGHT) && last_vx < 0)) {
        turn_dist_target = NEW_WALL_DISTANCE_SUP;
      } else { 
        turn_dist_target = NEW_WALL_DISTANCE_INF;
      }
      double turn_threshold = std::sqrt(last_wall_dist * last_wall_dist + turn_dist_target * turn_dist_target);
      
      bool lost_F = (hugged_wall == FRONT && ps_val[3] >= turn_threshold && ps_val[4] >= turn_threshold);
      bool lost_B = (hugged_wall == BACK  && ps_val[0] >= turn_threshold && ps_val[7] >= turn_threshold);
      bool lost_L = (hugged_wall == LEFT  && ps_val[5] >= turn_threshold && ps_val[1] >= turn_threshold); // LATERAL-I
      bool lost_R = (hugged_wall == RIGHT && ps_val[2] >= turn_threshold && ps_val[6] >= turn_threshold); // LATERAL-D

      if (lost_F || lost_B || lost_L || lost_R) {
        // Guardar punto de inicio del movimiento tras la esquina y
        // la distancia que queremos recorrer con la velocidad previa
        // antes de empezar a rodear la esquina.
        corner_start_x = pos_x;
        corner_start_y = pos_y;
        if (lost_B || (lost_L && last_vx < 0) || (lost_R && last_vx < 0)) {
          // Esquinas superiores del mapa
          corner_travel_target = NEW_WALL_DISTANCE_SUP;
        } else {
          // Esquinas inferiores del mapa
          corner_travel_target = NEW_WALL_DISTANCE_INF;
        }

        // Fase 0 de BUSCANDO_PARED: mantener velocidad previa
        // (recto) durante corner_travel_target metros
        corner_phase = 0;
        corner_pre_vx = last_vx;
        corner_pre_vy = last_vy;

        // Calcular velocidad de la fase 1 (rodear la esquina)
        double turn_vx = 0.0, turn_vy = 0.0;
        WallType new_hugged = hugged_wall;
        DirType new_dir = travel_dir;

        if (lost_B) { // Pared TRASERA (-X) ended. We are at Sup Map. Corners 1 or 2
          if (last_vy > 0) { // Iba hacia Drcha del mapa (+Y), passing 2 Sup Drcha Ext
            turn_vx = ROBOT_SPEED; turn_vy = 0; // Gira hacia Alante (+X) para bajar a Inf
            new_hugged = LEFT; new_dir = FORWARD;
          } else {           // Iba hacia Izqda del mapa (-Y), passing 1 Sup Izq Ext
            turn_vx = ROBOT_SPEED; turn_vy = 0; // Gira hacia Alante (+X) para bajar a Inf
            new_hugged = RIGHT; new_dir = FORWARD;
          }
        } else if (lost_F) { // Pared FRONTAL (+X) ended. We are at Inf Map. Corners 3 or 4
          if (last_vy > 0) { // Iba hacia Drcha del mapa (+Y), passing 4 Inf Drcha Ext
            turn_vx = -ROBOT_SPEED; turn_vy = 0; // Gira hacia Atras (-X) para subir a Sup
            new_hugged = LEFT; new_dir = BACKWARD;
          } else {           // Iba hacia Izqda del mapa (-Y), passing 3 Inf Izq Ext
            turn_vx = -ROBOT_SPEED; turn_vy = 0; // Gira hacia Atras (-X) para subir a Sup
            new_hugged = RIGHT; new_dir = BACKWARD;
          }
        } else if (lost_L) { // Pared LEFT ended.
          if (last_vx > 0) { // Iba hacia Alante (+X), passing 4 Inf Drcha Ext
            turn_vx = 0; turn_vy = +ROBOT_SPEED; // Gira hacia Izqda map (-Y)
            new_hugged = RIGHT; new_dir = MOVE_RIGHT; // Swapped from FRONT to RIGHT, and MOVE_RIGHT was already there.
          } else {           // Iba hacia Atras (-X), passing 1 Sup Izq Ext
            turn_vx = 0; turn_vy = -ROBOT_SPEED; // Gira hacia Drcha map (+Y)
            new_hugged = LEFT; new_dir = MOVE_LEFT; // Swapped from BACK to LEFT, and MOVE_LEFT was already there.
          }
        } else if (lost_R) { // Pared RIGHT (-Y) ended. Moving along Izqda Map. Corners 1 or 3
          if (last_vx > 0) { // Iba hacia Alante (+X), passing 3 Inf Izq Ext
            turn_vx = 0; turn_vy = -ROBOT_SPEED; // Gira hacia Drcha map (+Y)
            new_hugged = LEFT; new_dir = MOVE_LEFT; // Swapped from FRONT to LEFT, and MOVE_LEFT was already there.
          } else {           // Iba hacia Atras (-X), passing 1 Sup Izq Ext
            turn_vx = 0; turn_vy = -ROBOT_SPEED; // Gira hacia Drcha map (+Y)
            new_hugged = FRONT; new_dir = MOVE_LEFT; // Swapped from BACK to FRONT, and MOVE_LEFT was already there.
          }
        }

        // Guardar fase 1 (cómo rodear la esquina) y
        // la nueva pared que se pretende seguir después.
        corner_turn_vx = turn_vx;
        corner_turn_vy = turn_vy;
        corner_turn_dir = new_dir;
        next_hugged_wall = new_hugged;

        // Primera fase en BUSCANDO_PARED: misma velocidad que antes
        vx_w = corner_pre_vx;
        vy_w = corner_pre_vy;

        // Actualizar travel_dir coherente con la velocidad previa
        if (std::abs(vx_w) >= std::abs(vy_w)) {
          if (vx_w > 0)
            travel_dir = BACKWARD; // ver notas en sensores_vs_esquinas.txt
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

      // Seguimiento normal: bloquear eje de la pared, moverse continuamente
      // en el sentido paralelo que traíamos (travel_dir)
      if (hugged_wall == FRONT || hugged_wall == BACK) {
        vx_w = 0.0;
        if (travel_dir == MOVE_LEFT) {
          vy_w = ROBOT_SPEED;
        } else if (travel_dir == MOVE_RIGHT) {
          vy_w = -ROBOT_SPEED;
        } else {
          // Si por alguna razón no tenemos dirección perpendicular, escogemos una basándonos en el objetivo
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
          // Si no tenemos dirección, escogemos basándonos en el objetivo
          vx_w = (errX > 0) ? ROBOT_SPEED : -ROBOT_SPEED;
          travel_dir = (vx_w > 0) ? FORWARD : BACKWARD;
        }
      }
      break;
    }

    // ── ESQUINA INTERIOR ───────────────────────────────────────────
    case INT_CORNER:
    //{ VMP  
    if (wall_F && wall_L) { // 1 Sup izqda
         if (prev_hugged_wall == FRONT) {
             vx_w = ROBOT_SPEED; vy_w = 0; travel_dir = FORWARD;
             hugged_wall = LEFT;
         } else {
             vx_w = 0; vy_w = ROBOT_SPEED; travel_dir = MOVE_LEFT;
             hugged_wall = FRONT;
         }
      } else if (wall_B && wall_R) { // 4 Inf drcha
         if (prev_hugged_wall == BACK) {
             vx_w = -ROBOT_SPEED; vy_w = 0; travel_dir = BACKWARD;
             hugged_wall = RIGHT;
         } else {
             vx_w = 0; vy_w = -ROBOT_SPEED; travel_dir = MOVE_RIGHT;
             hugged_wall = BACK;
         }
      } else if (wall_F && wall_R) { // 2 Sup drcha
         if (prev_hugged_wall == FRONT) {
             vx_w = ROBOT_SPEED; vy_w = 0; travel_dir = FORWARD;
             hugged_wall = RIGHT;
         } else {
             vx_w = 0; vy_w = -ROBOT_SPEED; travel_dir = MOVE_RIGHT;
             hugged_wall = FRONT;
         }
      } else if (wall_B && wall_L) { // 3 Inf izqda
         if (prev_hugged_wall == BACK) {
             vx_w = -ROBOT_SPEED; vy_w = 0; travel_dir = BACKWARD;
             hugged_wall = LEFT;
         } else {
             vx_w = 0; vy_w = ROBOT_SPEED; travel_dir = MOVE_LEFT;
             hugged_wall = BACK;
         }
      }
      //} VMP

      // Salida del corner cuando dejamos de ver alguna de las paredes interiores
      if (!((wall_F && wall_L) || (wall_F && wall_R) || (wall_B && wall_L) || (wall_B && wall_R))) {
        current_state = SIGUIENDO_PARED;
      }
      break;

    // ── BUSCANDO PARED LUEGO DE DAR LA VUELTA A LA ESQUINA ─────────
    case FINDING_WALL: {
      // Fase 0: avanzar recto con la velocidad previa hasta recorrer
      // NEW_WALL_DISTANCE_xxx metros desde la esquina exterior.
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
      double check_limit = WALL_SIDE_LIMIT + 0.50; // Margen razonable para re-engancharse a la pared
      // La pared que queremos encontrar después de la esquina es next_hugged_wall,
      // no la que estábamos siguiendo antes (hugged_wall).
      if (next_hugged_wall == FRONT)
        see_wall = (ps_val[3] < check_limit || ps_val[4] < check_limit);
      else if (next_hugged_wall == BACK)
        see_wall = (ps_val[0] < check_limit || ps_val[7] < check_limit);
      else if (next_hugged_wall == LEFT)
        // Para reacoplar pared solo usamos el lateral puro, no el diagonal,
        // para evitar confundir paredes superiores con laterales.
        see_wall = (ps_val[5] < check_limit); // LATERAL-I (ps5)
      else if (next_hugged_wall == RIGHT)
        see_wall = (ps_val[2] < check_limit); // LATERAL-D (ps2)

      // Seguridad por si nos chocamos de frente con la nueva pared
      bool obstacle = false;
      if (travel_dir == FORWARD && wall_F) obstacle = true;
      if (travel_dir == BACKWARD && wall_B) obstacle = true;
      if (travel_dir == MOVE_LEFT && wall_L) obstacle = true;
      if (travel_dir == MOVE_RIGHT && wall_R) obstacle = true;

      // Una vez avista la nueva pared que quiere seguir (o se choca), retoma SIGUIENDO_PARED
      if (see_wall || obstacle) {
        // En este punto ya hemos encontrado realmente la nueva pared,
        // así que ahora sí actualizamos hugged_wall.
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

    // Transformar velocidades mundo → robot (rotación por yaw)
    twist.linear.x = vx_w * std::cos(yaw) + vy_w * std::sin(yaw);
    twist.linear.y = -vx_w * std::sin(yaw) + vy_w * std::cos(yaw);
    pub_vel->publish(twist);

    // Actualizar display cada 5 ticks (~500ms), como test_sensores.cpp con
    // frameCount%10
    if (tick_count % 5 == 0)
      draw_dashboard();
  }

  // -----------------------------------------------------------------------
  // Auxiliares
  // -----------------------------------------------------------------------


  void stop() { pub_vel->publish(geometry_msgs::msg::Twist()); }

  // -----------------------------------------------------------------------
  // Miembros
  // -----------------------------------------------------------------------
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

// ---------------------------------------------------------------------------
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ManhattanController>());
  rclcpp::shutdown();
  return 0;
}