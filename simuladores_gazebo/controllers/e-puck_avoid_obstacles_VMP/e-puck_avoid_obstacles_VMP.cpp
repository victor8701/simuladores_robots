// File:          e-puck_avoid_obstacles_VMP.cpp
// Date:          15/02/2026
// Description:   Controlador de e-puck con wall-following y Manhattan
// Author:        Victor Martin Parra
// Modifications: Añadidos extras de voz, LEDs y recuperacion

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/GPS.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/LED.hpp>
#include <webots/Speaker.hpp>

#include <iostream>
#include <iomanip>
#include <cmath>
#include <string>
#include <limits>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// Colores para output
#define COLOR_RESET   "\033[0m"
#define COLOR_GREEN   "\033[32m"
#define COLOR_YELLOW  "\033[33m"
#define COLOR_CYAN    "\033[36m"
#define COLOR_WHITE   "\033[37m"
#define COLOR_RED     "\033[31m"
#define COLOR_BLUE    "\033[34m"
#define COLOR_MAGENTA "\033[35m"

void printColored(const std::string& message, const std::string& color = COLOR_RESET) {
  std::cout << color << message << COLOR_RESET << std::endl;
}

// ==================== VELOCIDADES ====================
#define MAX_SPEED 5.0
#define TURN_SPEED 2.5

// ==================== UMBRALES DE SENSORES ====================
#define WALL_THRESHOLD 90.0
#define FRONT_WALL_THRESHOLD 190.0
#define EXTERN_CORNER_THRESHOLD 75
#define COLLISION_THRESHOLD 380.0

// ==================== DISTANCIAS Y TIMING ====================
#define SEPARATION_DISTANCE 0.01
#define PRE_TURN_DISTANCE 0.045
#define WALL_LOST_CONFIRM 1
#define ADVANCE_STEPS 20
#define STOP_CYCLES 5
#define INTERIOR_TURN_CYCLES 30
#define COLLISION_COUNT_TRIGGER 3
#define GOAL_RADIUS 0.2

// ==================== CONTROL ====================
#define CORRECTION_FACTOR 0.3

// ==================== LIMITES DEL MAPA ====================
#define MAX_X 10.9
#define MAX_Y 10.9
#define MIN_X 1.1
#define MIN_Y 1.1

// ==================== DISPLAY ====================
#define UPDATE_INTERVAL 15  // Actualizar pantalla cada N frames

// Variables globales de estado
bool followingWall = false;
bool wallOnRight = true;
bool goalReached = false;
int wallLostCounter = 0;
bool turningExterior = false;
bool advancingPreTurn = false;
double preTurnStartX = 0;
double preTurnStartY = 0;
double exteriorTargetYaw = 0;
int advancingAfterTurn = 0;
bool waitingDiagonalClear = false;
bool turningInterior = false;
int interiorPhase = 0;
int interiorCounter = 0;
bool collisionRecovery = false;
int recoveryPhase = 0;
double recoveryStartX = 0;
double recoveryStartY = 0;
double recoveryTargetYaw = 0;
int collisionCounter = 0;
int postTurnSuppressCounter = 0;
int recoveryType = 0;

// Dispositivos del robot
Motor* motor_left = nullptr;
Motor* motor_right = nullptr;
GPS* gps = nullptr;
InertialUnit* imu = nullptr;
DistanceSensor* ds[8];
LED* leds[10];  // 8 rojos + 1 RGB frontal + 1 body
Speaker* speaker = nullptr;

// Estado anterior para detectar cambios
std::string estadoAnterior = "";

double position_goal[3];

// Limpia pantalla y posiciona cursor al inicio
void clearScreen() {
  std::cout << "\033[2J\033[1;1H";
}

// Calcula la esquina diagonal opuesta como objetivo
void calcularPosicionObjetivo(const double position_ini[3]) {
  if (position_ini[0] < 6.0 && position_ini[1] < 6.0) {
    position_goal[0] = MAX_X;
    position_goal[1] = MAX_Y;
  } else if (position_ini[0] > 6.0 && position_ini[1] < 6.0) {
    position_goal[0] = MIN_X;
    position_goal[1] = MAX_Y;
  } else if (position_ini[0] < 6.0 && position_ini[1] > 6.0) {
    position_goal[0] = MAX_X;
    position_goal[1] = MIN_Y;
  } else {
    position_goal[0] = MIN_X;
    position_goal[1] = MIN_Y;
  }
  position_goal[2] = 0.0;
}

// Distancia entre dos puntos
double calcularDistancia(const double pos1[3], const double pos2[3]) {
  double dx = pos2[0] - pos1[0];
  double dy = pos2[1] - pos1[1];
  return std::sqrt(dx*dx + dy*dy);
}

// Activa el giro en esquina interior
void iniciarGiroInterior() {
  turningInterior = true;
  interiorPhase = 0;
  interiorCounter = 0;
  wallLostCounter = 0;
}

// Aplica velocidades con correccion de alineacion Manhattan
void moveRobot(double vL, double vR) {
  if (!imu) {
    motor_left->setVelocity(vL);
    motor_right->setVelocity(vR);
    return;
  }
  
  bool isTranslating = (vL > 0.1 && vR > 0.1);
  
  if (isTranslating) {
    const double* rpy = imu->getRollPitchYaw();
    double yaw = rpy[2] * 180.0 / M_PI;
    while(yaw < 0) yaw += 360.0;
    
    // Redondear a cardinal mas cercano
    double nearest = round(yaw / 90.0) * 90.0;
    if (nearest >= 360.0) nearest = 0.0;
    
    double error = nearest - yaw;
    while(error > 180.0) error -= 360.0;
    while(error < -180.0) error += 360.0;
    
    // Si no esta alineado, corregir con rotacion
    if (std::abs(error) > 1.0) {
      double kp = 0.5;
      double rotSpeed = std::abs(error) * kp;
      rotSpeed = std::max(0.2, std::min(2.0, rotSpeed));
      
      if (error > 0) {
        motor_left->setVelocity(-rotSpeed);
        motor_right->setVelocity(rotSpeed);
      } else {
        motor_left->setVelocity(rotSpeed);
        motor_right->setVelocity(-rotSpeed);
      }
      return;
    }
  }
  
  motor_left->setVelocity(vL);
  motor_right->setVelocity(vR);
}

// Lee sensores y actualiza contador de supresion
void leerSensores(double dsValues[8]) {
  if (postTurnSuppressCounter > 0) postTurnSuppressCounter--;
  for (int i = 0; i < 8; i++) {
    dsValues[i] = ds[i] ? ds[i]->getValue() : 0.0;
  }
}

// Detecta colision en cualquier sensor
bool detectarColision(const double dsValues[8]) {
  bool collisionRight = (dsValues[2] > COLLISION_THRESHOLD || dsValues[1] > COLLISION_THRESHOLD);
  bool collisionLeft = (dsValues[5] > COLLISION_THRESHOLD || dsValues[6] > COLLISION_THRESHOLD);
  bool collisionFront = (dsValues[0] > COLLISION_THRESHOLD || dsValues[7] > COLLISION_THRESHOLD);
  return collisionRight || collisionLeft || collisionFront;
}

// Gestiona contador de colisiones y activa recuperacion
void gestionarColisiones(bool isColliding, bool collisionFront) {
  if (isColliding) {
    collisionCounter++;
    if (collisionCounter >= COLLISION_COUNT_TRIGGER && !collisionRecovery) {
      collisionRecovery = true;
      recoveryPhase = 0;
      
      if (collisionFront) {
        recoveryType = 1;  // Retroceder y girar
      } else {
        recoveryType = 2;  // Separarse lateralmente
      }
      
      // Guardar orientacion cardinal actual
      const double* rpy = imu->getRollPitchYaw();
      double yaw = rpy[2] * 180.0 / M_PI;
      if (yaw < 0) yaw += 360;
      recoveryTargetYaw = round(yaw / 90.0) * 90.0;
      if (recoveryTargetYaw >= 360) recoveryTargetYaw = 0;
    }
  } else {
    collisionCounter = 0;
  }
}

// Verifica si se alcanzo el objetivo
bool verificarObjetivo(const double current_pos[3]) {
  double distanceToGoal = calcularDistancia(current_pos, position_goal);
  if (distanceToGoal < GOAL_RADIUS) {
    goalReached = true;
    motor_left->setVelocity(0.0);
    motor_right->setVelocity(0.0);
    std::cout << "\nObjetivo alcanzado en [" << current_pos[0] << ", " << current_pos[1] << "]" << std::endl;
    return true;
  }
  return false;
}

// Navegacion en espacio libre hacia objetivo
void navegacionLibre(const double current_pos[3], double& vel_left, double& vel_right) {
  double errX = position_goal[0] - current_pos[0];
  double errY = position_goal[1] - current_pos[1];
  
  // Elegir eje con menor error
  double targetYaw = 0;
  bool moveInX = (std::abs(errX) < std::abs(errY));
  
  if (moveInX) {
    if (errX > 0) targetYaw = 0.0;
    else targetYaw = 180.0;
  } else {
    if (errY > 0) targetYaw = 90.0;
    else targetYaw = 270.0;
  }
  
  // Correccion con IMU
  const double* rpy = imu->getRollPitchYaw();
  double currentYawDeg = rpy[2] * 180.0 / M_PI;
  if (currentYawDeg < 0) currentYawDeg += 360.0;
  
  double errorDeg = targetYaw - currentYawDeg;
  while (errorDeg > 180.0) errorDeg -= 360.0;
  while (errorDeg < -180.0) errorDeg += 360.0;
  
  double correction = errorDeg * CORRECTION_FACTOR;
  correction = std::max(-2.0, std::min(2.0, correction));
  
  vel_left = MAX_SPEED - correction;
  vel_right = MAX_SPEED + correction;
  
  vel_left = std::max(0.0, std::min(MAX_SPEED, vel_left));
  vel_right = std::max(0.0, std::min(MAX_SPEED, vel_right));
}

// Inicia seguimiento de pared al detectarla
void iniciarSeguimientoPared(bool wallRight, bool wallLeft, bool wallFront, const double current_pos[3]) {
  if (wallRight || wallLeft) {
    followingWall = true;
    wallOnRight = wallRight;
  } else if (wallFront) {
    followingWall = true;
    
    // Decidir lado de giro segun posicion del objetivo
    const double* rpy = imu->getRollPitchYaw();
    double heading = rpy[2];
    double goalAngle = atan2(position_goal[1] - current_pos[1], position_goal[0] - current_pos[0]);
    
    double diff = goalAngle - heading;
    while(diff > M_PI) diff -= 2*M_PI;
    while(diff < -M_PI) diff += 2*M_PI;
    
    wallOnRight = (diff > 0);
    
    // Activar recuperacion frontal (retroceder -> girar -> seguir)
    collisionRecovery = true;
    recoveryType = 1;
    recoveryPhase = 0;
    
    // Guardar orientacion cardinal actual
    double yaw = rpy[2] * 180.0 / M_PI;
    if (yaw < 0) yaw += 360;
    recoveryTargetYaw = round(yaw / 90.0) * 90.0;
    if (recoveryTargetYaw >= 360) recoveryTargetYaw = 0;
  }
}

// Ejecuta recuperacion de colision
// Tipo 1 (frontal): retroceder -> girar 90 al lado correcto -> seguir pared
// Tipo 2 (lateral): girar 90 alejandose -> avanzar -> girar 90 de vuelta -> seguir recto
void recuperacionColision(double& vel_left, double& vel_right) {
  const double* rpy = imu->getRollPitchYaw();
  double currentYaw = rpy[2] * 180.0 / M_PI;
  if (currentYaw < 0) currentYaw += 360;
  
  // Tipo 1: Colision frontal (retroceder y girar 90)
  if (recoveryType == 1) {
    if (recoveryPhase == 0) {
      const double* pos = gps->getValues();
      recoveryStartX = pos[0];
      recoveryStartY = pos[1];
      recoveryPhase = 1;
    } else if (recoveryPhase == 1) {
      // Retroceder
      vel_left = -2.0;
      vel_right = -2.0;
      
      const double* pos = gps->getValues();
      double dx = pos[0] - recoveryStartX;
      double dy = pos[1] - recoveryStartY;
      double dist = sqrt(dx*dx + dy*dy);
      
      if (dist >= SEPARATION_DISTANCE) {
        recoveryPhase = 2;
        // Calcular nuevo objetivo de orientacion
        double target = recoveryTargetYaw;
        if (wallOnRight) target += 90.0;
        else target -= 90.0;
        
        if (target >= 360.0) target -= 360.0;
        if (target < 0.0) target += 360.0;
        recoveryTargetYaw = target;
      }
    } else if (recoveryPhase == 2) {
      // Girar 90 grados
      double error = recoveryTargetYaw - currentYaw;
      if (error > 180) error -= 360;
      if (error < -180) error += 360;
      
      if (std::abs(error) < 2.0) {
        // Recuperacion completa
        collisionRecovery = false;
        recoveryPhase = 0;
        collisionCounter = 0;
        postTurnSuppressCounter = 20;
        wallLostCounter = 0;
      } else {
        double rotSpeed = 2.0;
        if (error > 0) {
          vel_left = -rotSpeed;
          vel_right = rotSpeed;
        } else {
          vel_left = rotSpeed;
          vel_right = -rotSpeed;
        }
      }
    }
  } 
  // Tipo 2: Colision lateral (alejarse perpendicular)
  else {
    if (recoveryPhase == 0) {
      // Girar perpendicular a la pared
      double perpYaw;
      if (wallOnRight) {
        perpYaw = fmod(recoveryTargetYaw + 90, 360.0);
      } else {
        perpYaw = fmod(recoveryTargetYaw - 90 + 360, 360.0);
      }
      
      double angleError = perpYaw - currentYaw;
      if (angleError > 180) angleError -= 360;
      if (angleError < -180) angleError += 360;
      
      if (std::abs(angleError) < 3) {
        recoveryPhase = 1;
        const double* pos = gps->getValues();
        recoveryStartX = pos[0];
        recoveryStartY = pos[1];
        vel_left = 0;
        vel_right = 0;
      } else {
        if (angleError > 0) {
          vel_left = -TURN_SPEED;
          vel_right = TURN_SPEED;
        } else {
          vel_left = TURN_SPEED;
          vel_right = -TURN_SPEED;
        }
      }
    } else if (recoveryPhase == 1) {
      // Avanzar para separarse
      const double* pos = gps->getValues();
      double dx = pos[0] - recoveryStartX;
      double dy = pos[1] - recoveryStartY;
      double distanceMoved = sqrt(dx*dx + dy*dy);
      
      if (distanceMoved >= SEPARATION_DISTANCE) {
        recoveryPhase = 2;
        vel_left = 0;
        vel_right = 0;
      } else {
        vel_left = MAX_SPEED * 0.5;
        vel_right = MAX_SPEED * 0.5;
      }
    } else if (recoveryPhase == 2) {
      // Girar de vuelta a orientacion original
      double angleError = recoveryTargetYaw - currentYaw;
      if (angleError > 180) angleError -= 360;
      if (angleError < -180) angleError += 360;
      
      if (std::abs(angleError) < 3) {
        collisionRecovery = false;
        recoveryPhase = 0;
        collisionCounter = 0;
        postTurnSuppressCounter = 20;
        wallLostCounter = 0;
        vel_left = 0;
        vel_right = 0;
      } else {
        if (angleError > 0) {
          vel_left = -TURN_SPEED;
          vel_right = TURN_SPEED;
        } else {
          vel_left = TURN_SPEED;
          vel_right = -TURN_SPEED;
        }
      }
    }
  }
}

// Avanza antes de girar en esquina exterior
void avancePreGiro(double& vel_left, double& vel_right) {
  const double* pos = gps->getValues();
  double dx = pos[0] - preTurnStartX;
  double dy = pos[1] - preTurnStartY;
  double dist = sqrt(dx*dx + dy*dy);
  
  if (dist >= PRE_TURN_DISTANCE) {
    advancingPreTurn = false;
    turningExterior = true;
    advancingAfterTurn = 0;
    waitingDiagonalClear = false;
    
    // Calcular orientacion objetivo para el giro
    const double* rpy = imu->getRollPitchYaw();
    double currentYaw = rpy[2] * 180.0 / M_PI;
    if (currentYaw < 0) currentYaw += 360.0;
    
    double cardinalYaw = round(currentYaw / 90.0) * 90.0;
    if (cardinalYaw >= 360.0) cardinalYaw = 0.0;
    
    if (wallOnRight) {
      exteriorTargetYaw = cardinalYaw - 90.0;
    } else {
      exteriorTargetYaw = cardinalYaw + 90.0;
    }
    
    if (exteriorTargetYaw >= 360.0) exteriorTargetYaw -= 360.0;
    if (exteriorTargetYaw < 0.0) exteriorTargetYaw += 360.0;
  } else {
    vel_left = MAX_SPEED;
    vel_right = MAX_SPEED;
  }
}

// Sigue la pared detectando esquinas interiores y exteriores
void seguimientoPared(bool wallFront, bool wallRight, bool wallLeft, const double dsValues[8], double& vel_left, double& vel_right) {
  // Pared frontal: esquina interior
  if (wallFront) {
    if (!turningInterior) {
      iniciarGiroInterior();
    }
  } 
  // Pared lateral presente: avanzar con correccion IMU
  else if ((wallOnRight && wallRight) || (!wallOnRight && wallLeft)) {
    wallLostCounter = 0;
    
    if (!collisionRecovery && imu != nullptr) {
      const double* rpy = imu->getRollPitchYaw();
      double current_yaw = rpy[2];
      double yaw_deg = current_yaw * 180.0 / M_PI;
      
      while (yaw_deg < 0) yaw_deg += 360.0;
      while (yaw_deg >= 360.0) yaw_deg -= 360.0;
      
      // Determinar cardinal mas cercano
      double target_yaw_deg;
      if (yaw_deg < 45.0 || yaw_deg >= 315.0) {
        target_yaw_deg = 0.0;
      } else if (yaw_deg >= 45.0 && yaw_deg < 135.0) {
        target_yaw_deg = 90.0;
      } else if (yaw_deg >= 135.0 && yaw_deg < 225.0) {
        target_yaw_deg = 180.0;
      } else {
        target_yaw_deg = 270.0;
      }
      
      double error_deg = target_yaw_deg - yaw_deg;
      while (error_deg > 180.0) error_deg -= 360.0;
      while (error_deg < -180.0) error_deg += 360.0;
      
      // Aplicar correccion proporcional
      double correction = error_deg * CORRECTION_FACTOR;
      correction = std::max(-2.0, std::min(2.0, correction));
      
      vel_left = MAX_SPEED - correction;
      vel_right = MAX_SPEED + correction;
      
      vel_left = std::max(0.0, std::min(MAX_SPEED, vel_left));
      vel_right = std::max(0.0, std::min(MAX_SPEED, vel_right));
    } else {
      vel_left = MAX_SPEED;
      vel_right = MAX_SPEED;
    }
  } 
  // Perdida de pared: posible esquina exterior
  else {
    bool lostTrackedWall = false;
    
    // Solo verificar si no estamos en supresion post-giro
    if (postTurnSuppressCounter <= 0) {
      if (wallOnRight && dsValues[2] < EXTERN_CORNER_THRESHOLD) {
        lostTrackedWall = true;
      } else if (!wallOnRight && dsValues[6] < EXTERN_CORNER_THRESHOLD) {
        lostTrackedWall = true;
      }
    }
    
    if (lostTrackedWall) {
      advancingPreTurn = true;
      const double* pos = gps->getValues();
      preTurnStartX = pos[0];
      preTurnStartY = pos[1];
      wallLostCounter = 0;
    } else {
      wallLostCounter = 0;
    }
    
    if (waitingDiagonalClear) {
      bool diagonalClear = false;
      if (wallOnRight && dsValues[1] < WALL_THRESHOLD) {
        diagonalClear = true;
      } else if (!wallOnRight && dsValues[5] < WALL_THRESHOLD) {
        diagonalClear = true;
      }
      
      if (diagonalClear) {
        turningExterior = true;
        waitingDiagonalClear = false;
      } else {
        vel_left = MAX_SPEED * 0.5;
        vel_right = MAX_SPEED * 0.5;
      }
    } else {
      vel_left = MAX_SPEED;
      vel_right = MAX_SPEED;
    }
  }
}

// Giro interior de 90 grados
void giroInterior(double& vel_left, double& vel_right) {
  if (interiorPhase == 0) {
    // Pausa antes de girar
    vel_left = 0;
    vel_right = 0;
    interiorCounter++;
    
    if (interiorCounter >= STOP_CYCLES) {
      interiorPhase = 1;
      interiorCounter = 0;
    }
  } else if (interiorPhase == 1) {
    // Giro de 90 grados
    if (wallOnRight) {
      vel_left = -TURN_SPEED;
      vel_right = TURN_SPEED;
    } else {
      vel_left = TURN_SPEED;
      vel_right = -TURN_SPEED;
    }
    
    interiorCounter++;
    if (interiorCounter >= INTERIOR_TURN_CYCLES) {
      turningInterior = false;
      interiorPhase = 0;
      interiorCounter = 0;
      postTurnSuppressCounter = 15;
      wallLostCounter = 0;
    }
  }
}

// Giro exterior de 90 grados
void giroExterior(double& vel_left, double& vel_right) {
  const double* rpy = imu->getRollPitchYaw();
  double currentYaw = rpy[2] * 180.0 / M_PI;
  if (currentYaw < 0) currentYaw += 360.0;
  
  double error = exteriorTargetYaw - currentYaw;
  if (error > 180.0) error -= 360.0;
  if (error < -180.0) error += 360.0;
  
  if (std::abs(error) < 2.0) {
    turningExterior = false;
    advancingAfterTurn = ADVANCE_STEPS;
    vel_left = MAX_SPEED;
    vel_right = MAX_SPEED;
  } else {
    if (error > 0) {
      vel_left = -TURN_SPEED;
      vel_right = TURN_SPEED;
    } else {
      vel_left = TURN_SPEED;
      vel_right = -TURN_SPEED;
    }
  }
}

// Avanza tras giro buscando nueva pared
void avancePostGiro(bool wallRight, bool wallLeft, double& vel_left, double& vel_right) {
  vel_left = MAX_SPEED;
  vel_right = MAX_SPEED;
  advancingAfterTurn--;
  
  if ((wallOnRight && wallRight) || (!wallOnRight && wallLeft)) {
    advancingAfterTurn = 0;
    wallLostCounter = 0;
  }
  
  if (advancingAfterTurn == 0) {
    wallLostCounter = 0;
  }
}

// Devuelve el estado actual
const char* obtenerEstado() {
  if (collisionRecovery) {
    if (recoveryType == 1) return "Recuperacion frontal";
    else return "Recuperacion lateral";
  }
  if (turningInterior) return "Giro interior";
  if (turningExterior) return "Giro exterior";
  if (advancingPreTurn) return "Pre-giro exterior";
  if (advancingAfterTurn > 0) return "Post-giro";
  if (followingWall) {
    if (wallOnRight) return "Pared derecha";
    else return "Pared izquierda";
  }
  return "Navegacion libre";
}

// Imprime estado actual del robot y sensores
void imprimirEstado(const double dsValues[8], const double current_pos[3]) {
  clearScreen();
  
  printColored("=== CONTROLADOR E-PUCK ===", COLOR_CYAN);
  std::cout << "Objetivo: [" << position_goal[0] << ", " << position_goal[1] << "]" << std::endl;
  std::cout << std::endl;
  
  std::cout << COLOR_YELLOW << "Estado: " << COLOR_RESET << obtenerEstado() << std::endl;
  std::cout << "Posicion: [" << std::fixed << std::setprecision(1) 
            << current_pos[0] << ", " << current_pos[1] << "]" << std::endl;
  std::cout << std::endl;
  
  printColored("Sensores:", COLOR_GREEN);
  
  // Colorear sensores segun valor
  auto printSensor = [](const char* name, int value) {
    std::string color = COLOR_WHITE;
    if (value > 300) color = COLOR_RED;
    else if (value > 150) color = COLOR_YELLOW;
    else if (value > 80) color = COLOR_GREEN;
    std::cout << "  " << name << ": " << color << std::setw(4) << value << COLOR_RESET << std::endl;
  };
  
  printSensor("ps0 (frontal izq)", (int)dsValues[0]);
  printSensor("ps1 (diag izq)   ", (int)dsValues[1]);
  printSensor("ps2 (lateral der)", (int)dsValues[2]);
  printSensor("ps5 (lateral izq)", (int)dsValues[5]);
  printSensor("ps6 (diag der)   ", (int)dsValues[6]);
  printSensor("ps7 (frontal der)", (int)dsValues[7]);
  std::cout << std::endl;
}

// Actualiza los LEDs rojos y el RGB según los sensores
void actualizarLEDs(const double dsValues[8]) {
  // 1. LEDs rojos periféricos (0-7): encender si sensor activo (>= 80)
  for (int i = 0; i < 8; i++) {
    if (leds[i]) {
      leds[i]->set(dsValues[i] >= 80 ? 1 : 0);
    }
  }
  
  // 2. LED RGB frontal (8): color más restrictivo
  if (leds[8]) {
    double maxVal = 0;
    for (int i = 0; i < 8; i++) {
      if (dsValues[i] > maxVal) maxVal = dsValues[i];
    }
    
    if (maxVal > COLLISION_THRESHOLD) leds[8]->set(0xFF0000);       // Rojo
    else if (maxVal > FRONT_WALL_THRESHOLD) leds[8]->set(0xFFFF00); // Amarillo
    else if (maxVal > WALL_THRESHOLD) leds[8]->set(0x00FF00);       // Verde
    else leds[8]->set(0x000000);                                   // Apagado
  }
}

// Reproduce un archivo de voz si el estado ha cambiado
void reproducirVozEstado(const char* nuevoEstado) {
  if (estadoAnterior != nuevoEstado) {
    if (speaker) {
      std::string archivo = "sounds/";
      std::string st = nuevoEstado;
      
      if (st == "Navegacion libre") archivo += "navegacion_libre.wav";
      else if (st == "Pared derecha") archivo += "pared_derecha.wav";
      else if (st == "Pared izquierda") archivo += "pared_izquierda.wav";
      else if (st == "Giro interior") archivo += "esquina_interior.wav";
      else if (st == "Giro exterior") archivo += "esquina_exterior.wav";
      else if (st == "Recuperacion frontal") archivo += "colision_frontal.wav";
      else if (st == "Recuperacion lateral") archivo += "colision_lateral.wav";
      else return; 
      
      // Usamos paplay para evitar problemas en WSL
      std::string comando = "paplay " + archivo + " &";
      system(comando.c_str());
      std::cout << "-> Reproduciendo: " << archivo << std::endl;
    }
    estadoAnterior = nuevoEstado;
  }
}

int main() {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = static_cast<int>(robot->getBasicTimeStep());
  
  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot.
  // Inicializar motores
  motor_left = robot->getMotor("left wheel motor");
  motor_right = robot->getMotor("right wheel motor");
  motor_left->setPosition(INFINITY);
  motor_right->setPosition(INFINITY);
  motor_left->setVelocity(0.0);
  motor_right->setVelocity(0.0);
  
  // Inicializar GPS
  gps = robot->getGPS("gps");
  if (gps) gps->enable(timeStep);
  
  // Inicializar IMU
  imu = robot->getInertialUnit("inertial unit");
  if (imu) imu->enable(timeStep);
  
  // Inicializar sensores de distancia
  const char* dsNames[8] = {"ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7"};
  for (int i = 0; i < 8; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    if (ds[i]) ds[i]->enable(timeStep);
  }
  
  // Inicializar LEDs
  for (int i = 0; i < 10; i++) {
    std::string ledName = "led" + std::to_string(i);
    leds[i] = robot->getLED(ledName);
  }
  
  // Inicializar Speaker
  speaker = robot->getSpeaker("speaker");
  if (speaker) {
    std::cout << "Dispositivo de sonido listo." << std::endl;
  } else {
    std::cout << "Aviso: No se encontro el speaker." << std::endl;
  }
  
  // Encender LED body (led9) al inicio
  if (leds[9]) leds[9]->set(1);
  
  robot->step(timeStep);
  
  // Calcular objetivo basado en posicion inicial
  const double* gps_values = gps->getValues();
  double position_ini[3] = {gps_values[0], gps_values[1], gps_values[2]};
  calcularPosicionObjetivo(position_ini);
  
  std::cout << "Controlador iniciado" << std::endl;
  std::cout << "Posicion inicial: [" << position_ini[0] << ", " << position_ini[1] << "]" << std::endl;
  std::cout << "Objetivo: [" << position_goal[0] << ", " << position_goal[1] << "]" << std::endl;
  std::cout << std::endl;
  
  int frameCount = 0;
  
  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(timeStep) != -1 && !goalReached) {
    frameCount++;
    
    // Read the sensors:
    double dsValues[8];
    leerSensores(dsValues);
    
    // Detectar paredes
    bool wallRight = (dsValues[2] > WALL_THRESHOLD);
    bool wallLeft = (dsValues[5] > WALL_THRESHOLD);
    bool wallFront = (dsValues[0] > FRONT_WALL_THRESHOLD || dsValues[7] > FRONT_WALL_THRESHOLD);
    
    // Detectar colisiones
    bool collisionRight = (dsValues[2] > COLLISION_THRESHOLD || dsValues[1] > COLLISION_THRESHOLD);
    bool collisionLeft = (dsValues[5] > COLLISION_THRESHOLD || dsValues[6] > COLLISION_THRESHOLD);
    bool collisionFront = (dsValues[0] > COLLISION_THRESHOLD || dsValues[7] > COLLISION_THRESHOLD);
    bool isColliding = collisionRight || collisionLeft || collisionFront;
    
    gestionarColisiones(isColliding, collisionFront);
    
    const double* current_pos = gps->getValues();
    if (verificarObjetivo(current_pos)) {
      // Apagar LED body al llegar al objetivo
      if (leds[9]) leds[9]->set(0);
      // Reproducir sonido de meta con paplay
      if (speaker) {
        system("paplay sounds/objetivo_alcanzado.wav &");
        std::cout << "-> Reproduciendo: sounds/objetivo_alcanzado.wav" << std::endl;
      }
      break;
    }
    
    double vel_left = 0.0;
    double vel_right = 0.0;
    
    // Decidir modo de navegacion
    if (!followingWall && !turningExterior && !advancingPreTurn && advancingAfterTurn == 0) {
      if (wallRight || wallLeft || wallFront) {
        iniciarSeguimientoPared(wallRight, wallLeft, wallFront, current_pos);
      } else {
        navegacionLibre(current_pos, vel_left, vel_right);
      }
    }
    
    // Ejecutar comportamiento actual
    if (collisionRecovery) {
      recuperacionColision(vel_left, vel_right);
    } else if (advancingPreTurn) {
      avancePreGiro(vel_left, vel_right);
    } else if (followingWall && !turningExterior && !turningInterior && !advancingPreTurn && advancingAfterTurn == 0) {
      seguimientoPared(wallFront, wallRight, wallLeft, dsValues, vel_left, vel_right);
    }
    
    // Ejecutar giros
    if (turningInterior) {
      giroInterior(vel_left, vel_right);
    } else if (turningExterior) {
      giroExterior(vel_left, vel_right);
    }
    
    // Avance post-giro
    if (advancingAfterTurn > 0) {
      avancePostGiro(wallRight, wallLeft, vel_left, vel_right);
    }
    
    // Actualizar display cada N frames
    if (frameCount % UPDATE_INTERVAL == 0) {
      imprimirEstado(dsValues, current_pos);
    }
    
    // Actualizar LEDs y voz
    actualizarLEDs(dsValues);
    reproducirVozEstado(obtenerEstado());
    
    // Aplicar velocidades calculadas
    moveRobot(vel_left, vel_right);
  }
  
  // Enter here exit cleanup code.
  std::cout << "Bye from c++!" << std::endl;

  delete robot;
  return 0;
}
