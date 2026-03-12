#!/usr/bin/env python3
"""
Práctica Gymnasium – Q-Learning en mapa CSV
============================================
Mueve un agente por Q-Learning desde el origen hasta la meta en un mapa CSV.

Uso:
    python3 practica_gymnasium.py            # mapa del profesor (map1, 12×12)
    python3 practica_gymnasium.py --mapa 2   # mapa mediano (map2, 20×20)
    python3 practica_gymnasium.py --mapa 3   # mapa grande   (map3, 30×30)

Sistema de coordenadas del entorno:
    X apunta hacia abajo (filas), Y apunta a la derecha (columnas).
    *--> Y (columnas)
    |
    v
    X (filas)
"""

import time
import argparse
import gymnasium as gym
import gymnasium_csv                                          # Entorno personalizado basado en CSV
from gymnasium.spaces import Discrete

class BoxToDiscreteObservation(gym.ObservationWrapper):
    def __init__(self, env):
        super().__init__(env)
        self.originalRange = (env.observation_space.high - env.observation_space.low)
        self.observation_space = Discrete(self.originalRange[0] * self.originalRange[1])

    def observation(self, obs):
        # Corrección: f * NUM_COLS + c (el original de gymnasium_csv era defectuoso)
        return obs[0] * self.originalRange[1] + obs[1]

import numpy as np
import matplotlib
matplotlib.use('Agg')       # Sin ventana gráfica extra (compatible con Pygame)
import matplotlib.pyplot as plt

# ---------------------------------------------------------------------------
# 0.  SELECCIÓN DE MAPA — argparse
# ---------------------------------------------------------------------------

# Catálogo de mapas disponibles.
# Cada mapa tiene su propio fichero CSV, posición inicial/meta y
# parámetros de entrenamiento ajustados a su tamaño y complejidad.
MAPAS = {
    1: dict(
        nombre        = 'Mapa del profesor  (12×12)',
        archivo       = 'map1.csv',
        initX=1,  initY=1,   # Esquina superior izquierda libre
        goalX=10, goalY=10,  # Esquina inferior derecha libre
        cols          = 12,
        epis          = 10000,   # Episodios de entrenamiento
        max_pasos     = 500,     # Pasos máximos por episodio
        epsilon_decay = 0.9995,  # ε × decay cada ep (llega a ~0.01 al final)
    ),
    2: dict(
        nombre        = 'Mapa mediano T    (20×20)',
        archivo       = 'map2.csv',
        initX=1,  initY=1,
        goalX=18, goalY=18,
        cols          = 20,
        # Diseño: pared vertical col 10 (filas 1-13) + brazo H fila 6 (cols 11-16)
        # Rutas: esquivar por cols 17-18 (arriba) o por hueco filas 14+ (abajo)
        epis          = 15000,
        max_pasos     = 600,
        epsilon_decay = 0.9997,
    ),
    3: dict(
        nombre        = 'Mapa grande C     (30×30)',
        archivo       = 'map3.csv',
        initX=1,  initY=1,
        goalX=28, goalY=28,
        cols          = 30,
        # Diseño: C-shape — V izq (col 8, filas 1-20) + H sup (fila 10, cols 9-22)
        #                  + V der (col 22, filas 11-20)
        # Rutas: esquivar por cols 23+ arriba (filas 1-9) o por filas 21+ abajo
        epis          = 30000,
        max_pasos     = 800,
        epsilon_decay = 0.99985,
    ),
    4: dict(
        nombre        = 'Mapa laberinto    (30×27)',
        archivo       = 'map4.csv',
        initX=1,  initY=1,
        goalX=25, goalY=28,  # Última fila libre, col cerca del borde derecho
        cols          = 30,   # 30 columnas en cada fila del CSV
        # Diseño: laberinto grid denso con muchos pasillos y giros
        # Requiere más episodios para explorar el laberinto
        epis          = 50000,
        max_pasos     = 1000,
        epsilon_decay = 0.999908,
    ),
}

parser = argparse.ArgumentParser(description='Q-Learning en mapa CSV con Gymnasium')
parser.add_argument(
    '--mapa', type=int, default=1, choices=[1, 2, 3, 4],
    help='Mapa a usar: 1=del profesor (12×12)  2=mediano-T (20×20)  3=grande-C (30×30)  4=laberinto (30×27)'
)
args = parser.parse_args()
cfg = MAPAS[args.mapa]   # Configuración del mapa elegido


# Extraemos todos los parámetros del mapa en variables locales para legibilidad
MAP_FILE      = cfg['archivo']
INIT_X, INIT_Y = cfg['initX'],  cfg['initY']
GOAL_X, GOAL_Y = cfg['goalX'],  cfg['goalY']
META_X, META_Y = GOAL_X, GOAL_Y   # Alias usados en el reward shaping
COLS          = cfg['cols']
epis          = cfg['epis']
max_pasos     = cfg['max_pasos']
epsilon_decay = cfg['epsilon_decay']

print(f"=== Mapa seleccionado: {cfg['nombre']} ===\n")

# ---------------------------------------------------------------------------
# 1. ENTORNO DE ENTRENAMIENTO (sin ventana gráfica, para que sea rápido)
# ---------------------------------------------------------------------------

env_train_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode=None,        # Sin visualización durante el entrenamiento (más rápido)
    inFileStr=MAP_FILE,
    initX=INIT_X, initY=INIT_Y,
    goalX=GOAL_X, goalY=GOAL_Y,
)
env_train = BoxToDiscreteObservation(env_train_raw)   # Obs (fila,col) → índice entero

# ---------------------------------------------------------------------------
# 2. INICIALIZACIÓN DE LA Q-TABLE
# ---------------------------------------------------------------------------

# Dimensiones: nS estados × nA acciones
# nS = filas × columnas del mapa  |  nA = 8 (8 direcciones cardinales + diagonales)
Q = np.ones([env_train.observation_space.n, env_train.action_space.n]) * 2.0

# ---------------------------------------------------------------------------
# 3. PARÁMETROS DEL ALGORITMO Q-LEARNING
# ---------------------------------------------------------------------------

eta            = 0.5    # Tasa de aprendizaje: cuánto actualiza Bellman en cada paso
gamma          = 0.95   # Factor de descuento: importancia de recompensas futuras

# ε-greedy con decaimiento EXPONENCIAL: empieza explorando 100% y decae hasta 1%
epsilon_inicio = 1.0
epsilon_fin    = 0.01
# epsilon_decay ya se cargó del catálogo de mapas

recompensas_por_episodio = []   # Historial de recompensas reales para la gráfica

# ---------------------------------------------------------------------------
# 4. BUCLE DE ENTRENAMIENTO
# ---------------------------------------------------------------------------

print(f"Entrenando con Q-Learning (sin ventana gráfica)...")
print(f"  Episodios: {epis}  |  eta={eta}  |  gamma={gamma}  |  ε: {epsilon_inicio}→{epsilon_fin} (exp)")
print("-" * 65)

epsilon = epsilon_inicio
intervalo_print = max(500, epis // 20)   # Imprimimos ~20 líneas de progreso

for episodio in range(epis):
    s, _ = env_train.reset()
    recompensa_total = 0
    terminado = False
    paso = 0

    # Distancia Manhattan inicial al objetivo (para reward shaping)
    fila_s, col_s = divmod(s, COLS)
    dist_ant = abs(fila_s - META_X) + abs(col_s - META_Y)

    while not terminado and paso < max_pasos:
        paso += 1

        # --- Selección de acción: ε-greedy con decaimiento exponencial ---
        # Con probabilidad ε tomamos una acción aleatoria (exploración).
        # Con probabilidad 1-ε tomamos la mejor acción conocida (explotación).
        if np.random.rand() < epsilon:
            accion = env_train.action_space.sample()   # Exploración uniforme
        else:
            accion = int(np.argmax(Q[s, :]))           # Explotación de la Q-table

        # --- Ejecutamos la acción en el entorno ---
        s1, recompensa, terminado, _, _ = env_train.step(accion)

        # --- Actualización Q-Table: regla de Bellman con Inicialización Optimista ---
        # Como Q empieza en 2.0 (muy optimista), el agente explorará sistemáticamente
        # las rutas desconocidas porque sus valores Q bajarán cuando no encuentren la meta.
        # Esto evita crear mínimos locales y hace innecesario el "reward shaping".
        
        # El futuro es nulo si chocamos contra un muro o llegamos a la meta
        futuro = 0.0 if terminado else np.max(Q[s1, :])
        
        # Penalización por cada paso dado (-0.01) para forzar el camino más corto
        recompensa_paso = (recompensa - 0.01) if not terminado else recompensa

        Q[s, accion] = Q[s, accion] + eta * (
            recompensa_paso + gamma * futuro - Q[s, accion]
        )

        recompensa_total += recompensa   # Guardamos recompensa REAL (sin shaping)
        s = s1

    recompensas_por_episodio.append(recompensa_total)
    epsilon = max(epsilon_fin, epsilon * epsilon_decay)   # Decay exponencial

    if (episodio + 1) % intervalo_print == 0:
        media = np.mean(recompensas_por_episodio[-intervalo_print:])
        print(f"  Ep {episodio+1:>6}/{epis}  |  Recompensa media: {media:.3f}  |  ε={epsilon:.4f}")

env_train.close()
print("-" * 65)
print(f"Entrenamiento completado. Recompensa media total: {np.mean(recompensas_por_episodio):.3f}")

# ---------------------------------------------------------------------------
# 4b. GRÁFICA DE CONVERGENCIA (curva de aprendizaje)
# ---------------------------------------------------------------------------

ventana = max(50, epis // 50)   # Ventana de la media móvil proporcional a los episodios
media_movil = np.convolve(
    recompensas_por_episodio,
    np.ones(ventana) / ventana,
    mode='valid'   # 'valid': solo valores donde la ventana está completa
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
    f'Convergencia Q-Learning — {cfg["nombre"]}  |  '
    f'η={eta}  γ={gamma}  ε-exp: {epsilon_inicio}→{epsilon_fin}'
)
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()

nombre_grafica = f'curva_aprendizaje_mapa{args.mapa}.png'   # Archivo distinto por mapa
plt.savefig(nombre_grafica, dpi=150)
print(f"\nGráfica de convergencia guardada en '{nombre_grafica}'")

# ---------------------------------------------------------------------------
# 5. ENTORNO DE DEMOSTRACIÓN (con ventana gráfica Pygame)
# ---------------------------------------------------------------------------

print("\nAbriendo entorno visual para mostrar la solución aprendida...")

import pygame

env_demo_raw = gym.make(
    'gymnasium_csv-v0',
    render_mode='human',   # Visualización con Pygame
    inFileStr=MAP_FILE,
    initX=INIT_X, initY=INIT_Y,
    goalX=GOAL_X, goalY=GOAL_Y,
)
env_demo = BoxToDiscreteObservation(env_demo_raw)

s, _ = env_demo.reset()
env_demo.render()   # Inicializa la ventana pygame (window, cellWidth, etc.)
time.sleep(1.0)

# ---------------------------------------------------------------------------
# Función de animación suave entre celdas
# ---------------------------------------------------------------------------
# Para movimientos diagonales próximos a muros, la interpolación en línea recta
# puede cruzar visualmente la esquina de la pared. En ese caso se usa una
# trayectoria en L (dos tramos rectos) para evitar el cruce visual.

def animar_movimiento(env_inner, pos_desde, pos_hasta, n_frames=20, fps=60):
    """
    Anima el robot de pos_desde a pos_hasta interpolando posiciones en píxeles.
    Detecta colisiones visuales con esquinas de muros y usa trayectoria en L.
    """
    win    = env_inner.window
    cw     = env_inner.cellWidth
    ch     = env_inner.cellHeight
    inFile = env_inner.inFile
    clock  = env_inner.clock

    def pixel_pos(fila, col):
        """Convierte (fila, col) en coordenadas píxel de la celda."""
        return np.array([col * cw, fila * ch], dtype=float)

    def dibujar_frame(pos_px):
        """Redibuja el mapa completo con el robot en pos_px (píxeles)."""
        canvas = pygame.Surface((env_inner.WINDOW_WIDTH, env_inner.WINDOW_HEIGHT))
        canvas.fill((0, 0, 0))
        for iX in range(inFile.shape[0]):
            for iY in range(inFile.shape[1]):
                if inFile[iX][iY] == 1:   # Muro → blanco
                    pygame.draw.rect(canvas, (255, 255, 255),
                                     pygame.Rect(cw*iY, ch*iX, cw, ch))
                if inFile[iX][iY] == 3:   # Meta → verde
                    pygame.draw.rect(canvas, (0, 255, 0),
                                     pygame.Rect(cw*iY, ch*iX, cw, ch))
        # Robot: rectángulo rojo centrado en la celda
        pygame.draw.rect(canvas, (255, 0, 0),
                         pygame.Rect(pos_px[0] + cw/4, pos_px[1] + ch/4, cw/2, ch/2))
        win.blit(canvas, canvas.get_rect())
        pygame.event.pump()
        pygame.display.update()
        clock.tick(fps)

    def interpolar(p_from, p_to, n):
        """Renderiza n+1 frames de interpolación lineal entre p_from y p_to."""
        for f in range(n + 1):
            t = f / n
            dibujar_frame(p_from + t * (p_to - p_from))

    dr = pos_hasta[0] - pos_desde[0]   # Δ fila
    dc = pos_hasta[1] - pos_desde[1]   # Δ columna

    if dr == 0 or dc == 0:
        # Movimiento recto (horizontal o vertical): interpolación directa
        interpolar(pixel_pos(*pos_desde), pixel_pos(*pos_hasta), n_frames)
    else:
        # Movimiento diagonal: comprobar si alguna esquina intermedia es muro
        esquina_a = (pos_hasta[0], pos_desde[1])   # (fila_dest, col_orig)
        esquina_b = (pos_desde[0], pos_hasta[1])   # (fila_orig, col_dest)
        a_muro = (inFile[esquina_a[0]][esquina_a[1]] == 1)
        b_muro = (inFile[esquina_b[0]][esquina_b[1]] == 1)
        mitad = n_frames // 2

        if a_muro and not b_muro:
            # Ir primero en columna (vía esquina_b), luego en fila
            interpolar(pixel_pos(*pos_desde),  pixel_pos(*esquina_b), mitad)
            interpolar(pixel_pos(*esquina_b),  pixel_pos(*pos_hasta), mitad)
        elif b_muro and not a_muro:
            # Ir primero en fila (vía esquina_a), luego en columna
            interpolar(pixel_pos(*pos_desde),  pixel_pos(*esquina_a), mitad)
            interpolar(pixel_pos(*esquina_a),  pixel_pos(*pos_hasta), mitad)
        else:
            # Ninguna esquina es muro: diagonal directa
            interpolar(pixel_pos(*pos_desde), pixel_pos(*pos_hasta), n_frames)

# ---------------------------------------------------------------------------
# 6. EJECUCIÓN DE LA SOLUCIÓN ENCONTRADA (paso a paso, con animación suave)
# ---------------------------------------------------------------------------

print("Ejecutando solución...")
terminado = False
paso = 0
env_inner = env_demo_raw.unwrapped   # GridWorldEnv sin wrappers (acceso a pygame)

while not terminado and paso < 200:
    paso += 1
    pos_antes   = env_inner._agent_location.copy()
    accion      = int(np.argmax(Q[s, :]))           # Mejor acción según Q-table
    s, recompensa, terminado, _, info = env_demo.step(accion)
    pos_despues = env_inner._agent_location.copy()

    # Animación suave: 20 frames a 60 fps (~0.33 s por paso)
    animar_movimiento(env_inner, pos_antes, pos_despues, n_frames=20, fps=60)

    print(f"  Paso {paso:>3}  |  Estado: {s:>4}  |  Recompensa: {recompensa:.1f}  |  Distancia a meta: {info['distance']:.1f}")

if terminado and recompensa > 0:
    print(f"\n¡Meta alcanzada en {paso} pasos! El agente ha aprendido a navegar el mapa.")
else:
    print("\nNo se alcanzó la meta en la demo. Prueba a aumentar 'epis'.")

time.sleep(2.0)
env_demo.close()
