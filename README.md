# Workspace de Simuladores de Robots

Este repositorio contiene tres proyectos distintos enfocados en la navegación autónoma de robots utilizando tres simuladores diferentes: **Webots**, **Gazebo** y **Gymnasium**. 

El objetivo general es aprender, diseñar e implementar algoritmos clave en la robótica móvil, tales como algoritmos reactivos (Follow-Wall) y aprendizaje por refuerzo (Q-Learning) para resolver laberintos dinámicos y evitar obstáculos.

## Estructura del Workspace

El workspace está dividido en tres carpetas principales, correspondiendo a cada uno de los enfoques:

### 1. `simuladores_webots`
**Tecnologías:** C, Webots Robot Simulator.
**Objetivo:** Control a bajo nivel y algoritmos reactivos.
**Descripción:** Implementa la navegación autónoma básica en un entorno 3D mediante C puro. Utiliza sensores de distancia infrarrojos para detectar paredes y algoritmos puramente reactivos para la evasión de colisiones.

### 2. `simuladores_gazebo`
**Tecnologías:** ROS 2 (Humble), C++, Gazebo Sim (Ignition).
**Objetivo:** Ecosistema completo de robótica modular y control en base a odometría.
**Descripción:** Construye una arquitectura moderna en ROS 2 orientada a nodos. Incluye descripciones de chasis con URDF, integración de un LIDAR de 8 haces, y un nodo en C++ que interpreta la odometría pura del mundo y el LIDAR simulado para recorrer esquinas exteriores/interiores limpiamente usando una máquina de estados compleja. Un vídeo completo demostrativo `video_gazebo.mp4` se encuentra en la raíz de esta subcarpeta.

### 3. `simuladores_gymnasium`
**Tecnologías:** Python 3, Gymnasium, Q-Learning, Numpy, PyGame.
**Objetivo:** Aprendizaje por Refuerzo (RL) e Inteligencia Artificial en grid-worlds discretos.
**Descripción:** Sustituye el enfoque manual y reactivo de código "if-else" por un cerebro inteligente capaz de resolver topografías laberínticas complejas aprendiendo en entornos 2D. Se entrena un agente con el algoritmo Q-Learning implementando una Tabla-Q dinámica y "Reward Shaping" basado en Breadth-First Search (BFS) para converger resolviendo desde los mapas más básicos de 10x10 hasta mapas masivos de 50x50 celdas, esquivando automáticamente todos los muros.

---
**Autor:** Victor Martin Parra
