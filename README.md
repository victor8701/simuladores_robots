# Simuladores Robots

Repositorio centralizado para proyectos de simulación en robótica. Este espacio está diseñado para albergar desarrollos en distintos simuladores, organizados por carpetas independientes.

## 📂 Estructura del Proyecto

*   **[simuladores_webots](./simuladores_webots)**: Contiene proyectos desarrollados en **Cyberbotics Webots R2023b**.
    *   `controllers/`: Controladores de robots (e-puck, etc.).
    *   `worlds/`: Archivos de escenario (.wbt).
*   **simuladores_gazebo** *(Próximamente)*: Espacio reservado para simulaciones en ROS/Gazebo.

---

## 🚀 Proyectos Destacados

### E-Puck Wall-Follower (Webots)
Controlador avanzado en C++ que implementa:
- Navegación Manhattan (alineación por IMU).
- Gestión de esquinas interiores y exteriores.
- Feedback por voz (WSL compatible) y LEDs RGB.
- Recuperación activa ante colisiones.

---

## 🛠️ Requisitos Generales
- Webots R2023a/b.
- Compilador GCC/G++ y Make.
- Herramientas de audio (`paplay`) para feedback sonoro.

Autor: [Victor Martin Parra](https://github.com/victor8701)
