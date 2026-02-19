# Proyecto E-Puck: Laberinto y Navegación

Controlador en C++ para el robot e-puck que navega autónomamente desde el origen hasta la meta mediante seguimiento de paredes.

## Para usar el proyecto:

1. **Carga del Mundo**: Abra Webots y cargue el archivo `/simuladores_webots/worlds/map.wbt`. Esto cargará el escenario completo con el robot e-puck configurado.
2. **Ubicación**: Asegúrese de que la carpeta `e-puck_avoid_obstacles_VMP` esté dentro del directorio `controllers/` para que Webots la asocie correctamente al robot.
    En mi caso tuve problemas para seleccionarlo, asi que edité el fichero map.wbt para añadir al final -> controller "e-puck_avoid_obstacles_VMP"
3. **Sonidos**: La carpeta `sounds/` debe permanecer junto al ejecutable.
4. **Requisitos**: El sistema de audio utiliza `paplay` (estándar en Linux/WSL). Si no hay audio, el controlador funcionará igualmente sin crashear.
5. **Compilación**:
   ```bash
   make clean
   make
   ```
6. **Ejecución**: Seleccione el controlador `e-puck_avoid_obstacles_VMP` en el nodo del robot dentro de Webots.

### Listado de Extras:

* **Extra (API):** Control de LEDs según <https://cyberbotics.com/doc/reference/led>. 8 LEDs rojos para proximidad local y frontal RGB según nivel de riesgo (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 701-721 y 776-780).

* **Extra (API):** Sistema de audio según <https://cyberbotics.com/doc/reference/speaker>. Feedback con voz femenina española para anunciar cambios de estado (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 723-746 y 782-788).

* **Extra (GUI):** Interfaz de consola mejorada con limpieza de pantalla, colores ANSI según criticidad y formateo de valores de sensores ps (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 18-30 y 668-698).

* **Extra (robótica):** Lógica avanzada de recuperación ante colisiones (frontal y lateral) permitiendo al robot salir de bloqueos ante obstáculos dinámicos (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 201-225 y 308-440).

* **Extra (robótica):** Navegación tipo Manhattan mediante IMU, obligando al robot a moverse únicamente en los ejes X e Y del mundo para mayor precisión (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 141-183 y 241-274).

* **Extra (robótica):** Algoritmo de gestión robusta de esquinas interiores y exteriores mediante máquina de estados y odometría para evitar pérdidas de pared (fichero e-puck_avoid_obstacles_VMP.cpp; líneas 478-648).