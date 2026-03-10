#!/usr/bin/env python3
"""
Práctica Gymnasium – Q-Learning en mapa CSV
============================================
Mueve un agente desde el origen (1,1) hasta la meta (10,10)
en un mapa 12x12 cargado desde un fichero CSV propio.

Sistema de coordenadas:
  X apunta hacia abajo (filas), Y apunta a la derecha (columnas).

  *--> Y (columnas)
  |
  v
  X (filas)
"""

import time
import gymnasium as gym
import gymnasium_csv                                          # Entorno personalizado basado en CSV
from gymnasium_csv.wrappers import BoxToDiscreteObservation  # Convierte observación Box -> discreta

import numpy as np

# ---------------------------------------------------------------------------
# 1. ENTORNO DE ENTRENAMIENTO (sin ventana gráfica, para que sea rápido)
# ---------------------------------------------------------------------------

# Usamos render_mode=None para que el entrenamiento no dibuje nada en pantalla.
# Esto acelera enormemente el proceso: sin Pygame, sin límite de fps.
env_train_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode=None,       # Sin visualización durante el entrenamiento
    inFileStr='map1.csv',   # Mapa con obstáculos en filas 4-5, columnas 1-3
    initX=1,                # Fila de inicio  (esquina superior izquierda libre)
    initY=1,                # Columna de inicio
    goalX=10,               # Fila de la meta  (esquina inferior derecha libre)
    goalY=10,               # Columna de la meta
)
# Aplicamos el wrapper para convertir la observación (fila, col) en un índice entero único.
env_train = BoxToDiscreteObservation(env_train_raw)

# ---------------------------------------------------------------------------
# 2. INICIALIZACIÓN DE LA Q-TABLE
# ---------------------------------------------------------------------------

# La Q-Table tiene una fila por cada estado posible y una columna por cada acción.
#   - nS = número de estados = filas * columnas del mapa
#   - nA = número de acciones = 8 (UP, UP_RIGHT, RIGHT, DOWN_RIGHT, DOWN, DOWN_LEFT, LEFT, UP_LEFT)
Q = np.zeros([env_train.observation_space.n, env_train.action_space.n])

# ---------------------------------------------------------------------------
# 3. PARÁMETROS DEL ALGORITMO Q-LEARNING
# ---------------------------------------------------------------------------

eta   = 0.628   # Tasa de aprendizaje (learning rate): cuánto actualizamos la Q-Table en cada paso
gamma = 0.9     # Factor de descuento: importancia de las recompensas futuras vs. inmediatas
epis  = 500     # Número de episodios de entrenamiento (aumentar si no converge)

recompensas_por_episodio = []   # Para seguir la evolución del entrenamiento

# ---------------------------------------------------------------------------
# 4. BUCLE DE ENTRENAMIENTO (rápido, sin renderizado)
# ---------------------------------------------------------------------------

print("Entrenando agente con Q-Learning (sin ventana gráfica para mayor velocidad)...")
print(f"  Episodios: {epis}  |  eta={eta}  |  gamma={gamma}")
print("-" * 55)

for episodio in range(epis):
    # Reiniciamos el entorno: el agente vuelve a la posición inicial (1,1)
    s, _ = env_train.reset()
    recompensa_total = 0
    terminado = False
    paso = 0

    while not terminado and paso < 200:   # Límite de pasos para evitar bucles infinitos
        paso += 1

        # --- Selección de acción (epsilon-greedy con exploración decreciente) ---
        # Al principio hay mucho ruido aleatorio (exploración).
        # A medida que avanzan los episodios, el ruido disminuye (explotación).
        ruido = np.random.randn(1, env_train.action_space.n) * (1.0 / (episodio + 1))
        accion = int(np.argmax(Q[s, :] + ruido))

        # --- Ejecutamos la acción en el entorno ---
        s1, recompensa, terminado, _, _ = env_train.step(accion)

        # --- Actualización de la Q-Table (regla de Bellman) ---
        # Q(s,a) += eta * [ r + gamma * max(Q(s')) - Q(s,a) ]
        Q[s, accion] = Q[s, accion] + eta * (
            recompensa + gamma * np.max(Q[s1, :]) - Q[s, accion]
        )

        recompensa_total += recompensa
        s = s1      # Avanzamos al siguiente estado

    recompensas_por_episodio.append(recompensa_total)

    # Imprimimos progreso cada 50 episodios
    if (episodio + 1) % 50 == 0:
        media = np.mean(recompensas_por_episodio[-50:])
        print(f"  Episodio {episodio+1:>4}/{epis}  |  Recompensa media (últ. 50): {media:.3f}")

env_train.close()   # Cerramos el entorno de entrenamiento (no tiene ventana, pero es buena práctica)

print("-" * 55)
print(f"Entrenamiento completado. Recompensa media total: {np.mean(recompensas_por_episodio):.3f}")

# ---------------------------------------------------------------------------
# 5. ENTORNO DE DEMOSTRACIÓN (con ventana gráfica para visualizar la solución)
# ---------------------------------------------------------------------------

print("\nAbriendo entorno visual para mostrar la solución aprendida...")

# Creamos un segundo entorno, esta vez con Pygame, para la demo visual final.
env_demo_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode='human',    # Visualización gráfica con Pygame
    inFileStr='map1.csv',
    initX=1,
    initY=1,
    goalX=10,
    goalY=10,
)
env_demo = BoxToDiscreteObservation(env_demo_raw)

# Reiniciamos y mostramos el estado inicial
s, _ = env_demo.reset()
env_demo.render()
time.sleep(1.0)     # Pausa para ver el estado inicial antes de que empiece a moverse

# ---------------------------------------------------------------------------
# 6. EJECUCIÓN DE LA SOLUCIÓN ENCONTRADA (paso a paso, visual)
# ---------------------------------------------------------------------------

print("Ejecutando solución...")
terminado = False
paso = 0

while not terminado and paso < 200:
    paso += 1

    # Elegimos la mejor acción según la Q-Table entrenada (sin exploración aleatoria)
    accion = int(np.argmax(Q[s, :]))

    # Ejecutamos el paso y renderizamos el movimiento en Pygame
    s, recompensa, terminado, _, info = env_demo.step(accion)
    env_demo.render()

    print(f"  Paso {paso:>3}  |  Estado: {s:>4}  |  Recompensa: {recompensa:.1f}  |  Distancia a meta: {info['distance']:.1f}")
    time.sleep(0.3)     # Pausa entre pasos para que la animación sea visible

# Mostramos el resultado final
if terminado and recompensa > 0:
    print("\n¡Meta alcanzada! El agente ha aprendido a navegar el mapa.")
else:
    print("\nNo se alcanzó la meta en la demo. Prueba a aumentar 'epis'.")

time.sleep(2.0)     # Pausa final para ver la posición de llegada
env_demo.close()
