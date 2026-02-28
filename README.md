# Laberinto Robot Gazebo

Simulación de navegación autónoma de un robot diferencial en Gazebo usando ROS 2. El robot navega desde su posición inicial hasta la esquina diagonalmente opuesta del mapa, implementando un algoritmo de seguimiento de paredes.

## Contenido

- **my_node_cpp**: Controlador de navegación
- **my_diffdrive_bringup**: Configuración de lanzamiento
- **my_diffdrive_description**: Descripción URDF del robot

## Funcionalidad

El robot sigue un algoritmo Manhattan que:
- Determina automáticamente el objetivo según su posición inicial
- Detecta y navega alrededor de obstáculos
- Maneja corners interiores y exteriores
- Reorienta hacia la pared más cercana tras girar esquinas

## Sensores Utilizados

- **8 sensores ultrasónicos** (LaserScan): detección de obstáculos en tiempo real
- **Odometría**: proporciona posición y orientación (yaw) del robot
- **Sensores de contacto**: para recuperación ante colisiones

## Extras Implementados

Dashboard interactivo con visualización en tiempo real de sensores ultrasónicos, máquina de estados con 5 estados (LIBRE, SIGUIENDO_PARED, INT_CORNER, FINDING_WALL, META_ALCANZADA), detección geométrica de esquinas, transformación de coordenadas mundo-robot.

## Requisitos Gazebo

- Gazebo Sim 7.0+ con soporte para plugins de sensores
- Modelos de robot con diferentialDrive y sensorPlugins configurados
- Topics de LaserScan para los 8 sensores ultrasónicos

## Requisitos del Sistema

- ROS 2 Humble
- Gazebo Sim
- Compilador GCC/G++ con C++17
- CMake 3.16+, colcon

Autor: Victor Martin Parra
