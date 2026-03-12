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
import matplotlib
matplotlib.use('Agg')       # Sin ventana gráfica interactiva (compatible con cualquier entorno)
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 1. ENTORNO DE ENTRENAMIENTO (sin ventana gráfica, para que sea rápido)
# ---------------------------------------------------------------------------

env_train_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode=None,
    inFileStr='map1.csv',
    initX=1,
    initY=1,
    goalX=10,
    goalY=10,
)
env_train = BoxToDiscreteObservation(env_train_raw)

# ---------------------------------------------------------------------------
# 2. INICIALIZACIÓN DE LA Q-TABLE
# ---------------------------------------------------------------------------

Q = np.zeros([env_train.observation_space.n, env_train.action_space.n])

# ---------------------------------------------------------------------------
# 3. PARÁMETROS DEL ALGORITMO Q-LEARNING
# ---------------------------------------------------------------------------

eta            = 0.5      # Tasa de aprendizaje
gamma          = 0.95     # Factor de descuento (valora más las recompensas futuras)
epis           = 10000    # Episodios de entrenamiento
max_pasos      = 500      # Pasos máximos por episodio

# ε-greedy con decaimiento EXPONENCIAL
epsilon_inicio = 1.0
epsilon_fin    = 0.01
epsilon_decay  = 0.9995   # ε = ε * decay cada episodio

# Geometría del mapa (para reward shaping)
META_X = 10
META_Y = 10
COLS   = 12   # Columnas del mapa CSV

recompensas_por_episodio = []

# ---------------------------------------------------------------------------
# 4. BUCLE DE ENTRENAMIENTO
# ---------------------------------------------------------------------------

print("Entrenando agente con Q-Learning (sin ventana gráfica para mayor velocidad)...")
print(f"  Episodios: {epis}  |  eta={eta}  |  gamma={gamma}  |  ε: {epsilon_inicio}→{epsilon_fin} (exp)")
print("-" * 65)

epsilon = epsilon_inicio

for episodio in range(epis):
    s, _ = env_train.reset()
    recompensa_total = 0
    terminado = False
    paso = 0

    # Distancia manhattan inicial al objetivo (para reward shaping)
    fila_s, col_s = divmod(s, COLS)
    dist_ant = abs(fila_s - META_X) + abs(col_s - META_Y)

    while not terminado and paso < max_pasos:
        paso += 1

        # --- Selección de acción: ε-greedy con decaimiento exponencial ---
        if np.random.rand() < epsilon:
            accion = env_train.action_space.sample()   # Exploración uniforme
        else:
            accion = int(np.argmax(Q[s, :]))           # Explotación

        # --- Ejecutamos la acción ---
        s1, recompensa, terminado, _, _ = env_train.step(accion)

        # --- Reward shaping: bonificación por acercarse a la meta ---
        # Proporciona gradiente de aprendizaje incluso antes de encontrar la meta.
        # shaping = +0.1 al acercarse, -0.1 al alejarse.
        fila_s1, col_s1 = divmod(s1, COLS)
        dist_nueva = abs(fila_s1 - META_X) + abs(col_s1 - META_Y)
        shaping = (dist_ant - dist_nueva) * 0.1
        dist_ant = dist_nueva

        # --- Actualización Q-Table (Bellman con recompensa moldeada) ---
        Q[s, accion] = Q[s, accion] + eta * (
            (recompensa + shaping) + gamma * np.max(Q[s1, :]) - Q[s, accion]
        )

        recompensa_total += recompensa   # Recompensa REAL (sin shaping) para la gráfica
        s = s1

    recompensas_por_episodio.append(recompensa_total)

    # Decaimiento exponencial de epsilon
    epsilon = max(epsilon_fin, epsilon * epsilon_decay)

    # Progreso cada 500 episodios
    if (episodio + 1) % 500 == 0:
        media = np.mean(recompensas_por_episodio[-500:])
        print(f"  Episodio {episodio+1:>5}/{epis}  |  Recompensa media (últ. 500): {media:.3f}  |  ε={epsilon:.4f}")

env_train.close()

print("-" * 65)
print(f"Entrenamiento completado. Recompensa media total: {np.mean(recompensas_por_episodio):.3f}")

# ---------------------------------------------------------------------------
# 4b. GRÁFICA DE CONVERGENCIA (curva de aprendizaje)
# ---------------------------------------------------------------------------

ventana = 200
media_movil = np.convolve(
    recompensas_por_episodio,
    np.ones(ventana) / ventana,
    mode='valid'
)

fig, ax = plt.subplots(figsize=(12, 5))

ax.plot(recompensas_por_episodio, color='lightsteelblue', alpha=0.3, label='Recompensa por episodio')
ax.plot(
    range(ventana - 1, epis),
    media_movil,
    color='steelblue', linewidth=2.5, label=f'Media móvil (ventana={ventana})'
)

ax.set_xlabel('Episodio')
ax.set_ylabel('Recompensa acumulada (real, sin shaping)')
ax.set_title(
    f'Curva de convergencia Q-Learning  |  η={eta}  γ={gamma}  '
    f'ε-exp: {epsilon_inicio}→{epsilon_fin}  |  {epis} episodios'
)
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('curva_aprendizaje.png', dpi=150)
print(f"\nGráfica de convergencia guardada en 'curva_aprendizaje.png'")

# ---------------------------------------------------------------------------
# 5. ENTORNO DE DEMOSTRACIÓN (con ventana gráfica)
# ---------------------------------------------------------------------------

print("\nAbriendo entorno visual para mostrar la solución aprendida...")

import pygame

env_demo_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode='human',
    inFileStr='map1.csv',
    initX=1,
    initY=1,
    goalX=10,
    goalY=10,
)
env_demo = BoxToDiscreteObservation(env_demo_raw)

s, _ = env_demo.reset()
env_demo.render()   # Inicializa la ventana pygame (window, cellWidth, etc.)
time.sleep(1.0)

# ---------------------------------------------------------------------------
# Función de animación suave entre dos celdas
# ---------------------------------------------------------------------------
# Dibuja el robot moviéndose gradualmente de una celda a otra en pygame,
# interpolando la posición en píxeles frame a frame.

def animar_movimiento(env_inner, pos_desde, pos_hasta, n_frames=20, fps=60):
    """
    Anima el robot moviéndose de pos_desde a pos_hasta.
    - Movimientos rectos: interpolación lineal directa.
    - Movimientos diagonales: si alguna celda de esquina es muro, hace dos tramos
      en L para no cruzar visualmente la pared.
    """
    win    = env_inner.window
    cw     = env_inner.cellWidth
    ch     = env_inner.cellHeight
    inFile = env_inner.inFile
    clock  = env_inner.clock

    def pixel_pos(fila, col):
        return np.array([col * cw, fila * ch], dtype=float)

    def dibujar_frame(pos_px):
        canvas = pygame.Surface((env_inner.WINDOW_WIDTH, env_inner.WINDOW_HEIGHT))
        canvas.fill((0, 0, 0))
        for iX in range(inFile.shape[0]):
            for iY in range(inFile.shape[1]):
                if inFile[iX][iY] == 1:
                    pygame.draw.rect(canvas, (255, 255, 255),
                                     pygame.Rect(cw*iY, ch*iX, cw, ch))
                if inFile[iX][iY] == 3:
                    pygame.draw.rect(canvas, (0, 255, 0),
                                     pygame.Rect(cw*iY, ch*iX, cw, ch))
        pygame.draw.rect(canvas, (255, 0, 0),
                         pygame.Rect(pos_px[0] + cw/4, pos_px[1] + ch/4,
                                     cw/2, ch/2))
        win.blit(canvas, canvas.get_rect())
        pygame.event.pump()
        pygame.display.update()
        clock.tick(fps)

    def interpolar(p_from, p_to, n):
        for f in range(n + 1):
            t = f / n
            dibujar_frame(p_from + t * (p_to - p_from))

    dr = pos_hasta[0] - pos_desde[0]   # diferencia de fila
    dc = pos_hasta[1] - pos_desde[1]   # diferencia de columna
    es_diagonal = (dr != 0 and dc != 0)

    if not es_diagonal:
        # Movimiento recto: interpolación directa
        interpolar(pixel_pos(*pos_desde), pixel_pos(*pos_hasta), n_frames)
    else:
        # Movimiento diagonal: comprobar esquinas intermedias
        esquina_a = (pos_hasta[0], pos_desde[1])   # fila destino, col origen
        esquina_b = (pos_desde[0], pos_hasta[1])   # fila origen,  col destino

        a_es_muro = (inFile[esquina_a[0]][esquina_a[1]] == 1)
        b_es_muro = (inFile[esquina_b[0]][esquina_b[1]] == 1)

        mitad = n_frames // 2

        if a_es_muro and not b_es_muro:
            # Ir primero en fila (via b), luego en columna
            interpolar(pixel_pos(*pos_desde), pixel_pos(*esquina_b), mitad)
            interpolar(pixel_pos(*esquina_b), pixel_pos(*pos_hasta), mitad)
        elif b_es_muro and not a_es_muro:
            # Ir primero en columna (via a), luego en fila
            interpolar(pixel_pos(*pos_desde), pixel_pos(*esquina_a), mitad)
            interpolar(pixel_pos(*esquina_a), pixel_pos(*pos_hasta), mitad)
        else:
            # Ninguna esquina es muro (o ambas): diagonal directa
            interpolar(pixel_pos(*pos_desde), pixel_pos(*pos_hasta), n_frames)


# ---------------------------------------------------------------------------
# 6. EJECUCIÓN DE LA SOLUCIÓN ENCONTRADA (con animación suave)
# ---------------------------------------------------------------------------

print("Ejecutando solución...")
terminado = False
paso = 0
env_inner = env_demo_raw.unwrapped   # Acceso al GridWorldEnv sin wrappers

while not terminado and paso < 200:
    paso += 1
    pos_antes  = env_inner._agent_location.copy()
    accion     = int(np.argmax(Q[s, :]))
    s, recompensa, terminado, _, info = env_demo.step(accion)
    pos_despues = env_inner._agent_location.copy()

    # 20 frames @ 60 fps ≈ 0.33 s de animación suave por paso
    animar_movimiento(env_inner, pos_antes, pos_despues, n_frames=20, fps=60)

    print(f"  Paso {paso:>3}  |  Estado: {s:>4}  |  Recompensa: {recompensa:.1f}  |  Distancia a meta: {info['distance']:.1f}")

if terminado and recompensa > 0:
    print("\n¡Meta alcanzada! El agente ha aprendido a navegar el mapa.")
else:
    print("\nNo se alcanzó la meta en la demo. Prueba a aumentar 'epis'.")

time.sleep(2.0)
env_demo.close()

