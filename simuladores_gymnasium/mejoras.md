# Opciones de implementación — Práctica Gymnasium (Q-Learning y RL)

Guía de estudio sobre qué técnicas implementar, ordenadas según **relevancia industrial actual** (2024-2026)
y **viabilidad real en este entorno discreto CSV**.

---

## Estado actual del script

| Elemento | Implementado |
|---|---|
| Entorno Gymnasium CSV personalizado | ✅ |
| Q-Learning tabular con epsilon-greedy | ✅ |
| Demo visual con Pygame | ✅ |
| Comentarios inline | ✅ |
| Gráficas / métricas / extras | ❌ |

---

## Contexto: ¿qué se usa realmente en industria hoy?

Los algoritmos más desplegados en robótica industrial, videojuegos y sistemas autónomos en 2024-2026 son:

| Algoritmo | Empresas que lo usan | Tipo |
|---|---|---|
| **PPO** (Proximal Policy Optimization) | OpenAI, NVIDIA Isaac Lab, Boston Dynamics | Policy Gradient, continuo |
| **SAC** (Soft Actor-Critic) | Universal Robots, DeepMind, Anybotics | Actor-Critic, continuo |
| **DQN / Double DQN** | videojuegos, trading HFT, logística | Value-based, discreto ✅ |
| **SARSA** | sistemas on-line críticos (seguridad) | Value-based, discreto ✅ |
| **Reward Shaping** | *todo el sector* (técnica, no algoritmo) | Transversal ✅ |

> PPO y SAC **no son viables aquí directamente** — requieren espacio de acciones continuo o aproximación de función (redes neuronales). Lo que sí encaja perfectamente con este entorno discreto son DQN, SARSA y Reward Shaping.

---

## Técnicas ordenadas de MEJOR a PEOR opción para esta práctica

---

### 🥇 1. SARSA (on-policy TD) — La más recomendada

> **Uso industrial actual**: robótica en producción donde el agente aprende *mientras opera* (líneas de ensamblaje, AGVs en almacenes). Es el algoritmo elegido cuando un error durante el entrenamiento tiene coste real.
> **Por qué es la mejor para esta práctica**: Cambia solo 3 líneas del script actual. Enseña el concepto on-policy vs off-policy, que es fundamental en RL moderno (SARSA → SAC es el mismo salto conceptual que Q-Learning → PPO). Compararla con Q-Learning en el mismo mapa da datos para justificar la elección.

**Diferencia clave con Q-Learning:**

| | Q-Learning (off-policy) | SARSA (on-policy) |
|---|---|---|
| Actualización | `max Q(s')` — acción óptima teórica | `Q(s', a')` — acción real tomada |
| Política | Aprende la óptima aunque explore aleatoriamente | Aprende la política que realmente ejecuta |
| Riesgo | Sobreestima valores en estados peligrosos | Más conservador, evita bordes de precipicio |
| Industria | Simulación, planificación offline | Robots en producción, sistemas críticos |

```python
# SARSA: elegir la siguiente acción ANTES de actualizar (on-policy)
epsilon = max(0.01, 1.0 - episodio / epis)   # ε decreciente
a1 = np.argmax(Q[s1, :]) if np.random.rand() > epsilon else env.action_space.sample()
Q[s, a] += eta * (r + gamma * Q[s1, a1] - Q[s, a])
s, a = s1, a1
```

---

### 🥈 2. Curva de aprendizaje + ε-decay — Rápida pero imprescindible

> **Uso industrial actual**: *toda* práctica real de RL incluye curvas de convergencia. Sin ellas no puedes saber si el agente ha convergido, si está sobre-explorando o si los hiperparámetros son correctos.
> **Por qué antes que le resto**: 30 minutos de trabajo, y **multiplica el valor de cualquier otro extra** — si implementas SARSA sin curva, no puedes compararlo con Q-Learning. Si haces Reward Shaping sin curva, no puedes ver si funciona.

```python
import matplotlib.pyplot as plt

# ε-decay: exploración alta al inicio, explotación al final (estándar industrial)
epsilon = max(0.01, 1.0 - episodio / epis)

# Al final del entrenamiento:
plt.plot(np.convolve(recompensas_por_episodio, np.ones(50)/50, mode='valid'))
plt.xlabel('Episodio')
plt.ylabel('Recompensa media (ventana 50)')
plt.title('Curva de convergencia Q-Learning')
plt.savefig('curva_aprendizaje.png')
```

---

### 🥉 3. Reward Shaping — Muy usado, pero con trampa en este entorno

> **Uso industrial actual**: *la técnica más usada en robótica real*. Boston Dynamics, NVIDIA Isaac Lab y cualquier empresa que entrene robots físicos diseña funciones de recompensa personalizadas. La recompensa nativa (llegaste/no llegaste) es demasiado escasa para convergencia rápida.
> **Advertencia importante para esta práctica**: el entorno `gymnasium-csv` encapsula las coordenadas. No tienes acceso directo a `x`, `y` — tienes que extraerlos del dict `info` que devuelve `step()`, concretamente `info['distance']`. El snippet naïve con `x_prev` requiere guardar el estado anterior manualmente.

```python
# Reward Shaping usando el distance del info dict (disponible en gymnasium-csv)
s1, r, terminado, _, info = env.step(accion)
distancia_nueva     = info['distance']
distancia_anterior  = distancia_guardada   # guardar en variable externa
r_shaped = r + 0.5 * (distancia_anterior - distancia_nueva)   # bonus por acercarse
distancia_guardada  = distancia_nueva
```

---

### 4. Double Q-Learning — Estándar de la industria, efecto limitado aquí

> **Uso industrial actual**: mejora estándar incluida en prácticamente todos los frameworks de DQN (Stable-Baselines3, RLlib, Dopamine). Es el default en producción sobre Q-Learning clásico.
> **Honestidad**: en un mapa 12×12 con recompensas simples, **el sesgo de sobreestimación es negligible**. Funciona, es correcto implementarlo, pero no verás diferencia empírica notable — lo cual dificulta justificarlo con datos en la memoria de la práctica.

```python
# Double Q-Learning: dos tablas cruzadas
if np.random.rand() < 0.5:
    a_best = np.argmax(Q_A[s1, :])
    Q_A[s, a] += eta * (r + gamma * Q_B[s1, a_best] - Q_A[s, a])
else:
    a_best = np.argmax(Q_B[s1, :])
    Q_B[s, a] += eta * (r + gamma * Q_A[s1, a_best] - Q_B[s, a])
```

---

### 5. Mejoras operativas menores

| Mejora | Descripción | Esfuerzo |
|---|---|---|
| **Múltiples mapas** | Entrenar en 2-3 CSV distintos y comparar pasos hasta meta | ⭐⭐ |
| **Hiperparámetro sweep** | Barrer `η` y `γ`, mostrar efecto en convergencia | ⭐⭐ |
| **Guardar/cargar Q-Table** | `np.save` / `np.load` para no reentrenar | ⭐ |
| **`argparse`** | Pasar mapa, episodios, eta, gamma por CLI | ⭐ |

---

### 6. Policy Gradient (REINFORCE) — Descartada para esta práctica

> **Uso industrial actual**: es el estándar absoluto hoy (PPO, SAC, RLHF en LLMs) pero **no es viable aquí**. Requiere función de aproximación (red neuronal), framework adicional (PyTorch/TensorFlow) y espacio de estados representado como vector, no como índice entero. El esfuerzo de adaptación supera con creces el beneficio en nota.

---

## Tabla resumen — ordenada por ratio impacto/esfuerzo

| Opción | Tipo | Esfuerzo | Impacto en nota | Uso industrial real |
|---|---|---|---|---|
| Curva de aprendizaje + ε-decay | Gymnasium | 30 min | ⭐⭐⭐ Alto | ✅ Imprescindible en cualquier proyecto |
| SARSA + comparativa | Gymnasium | 1–2h | ⭐⭐⭐ Muy alto | ✅ Robótica en producción |
| Reward Shaping | Gymnasium | 1h | ⭐⭐⭐ Alto | ✅ Universal en robótica real |
| Double Q-Learning | Gymnasium | 1–2h | ⭐⭐ Medio-alto | ✅ Estándar DQN, efecto limitado aquí |
| Múltiples mapas | Gymnasium | 1h | ⭐⭐ Medio-alto | ✅ Validación estándar |
| `argparse` + guardar Q-Table | Prog. general | 40 min | ⭐ Medio | ✅ Ingeniería de software básica |

---

## Referencias

- Sutton & Barto — *Reinforcement Learning: An Introduction* (cap. 6-7): Q-Learning y SARSA
- [Gymnasium docs](https://gymnasium.farama.org/) — API oficial
- [DeepMind DQN paper (2015)](https://www.nature.com/articles/nature14236) — Double DQN y replay buffer
- [OpenAI Spinning Up](https://spinningup.openai.com/) — guía práctica de PPO, SAC y REINFORCE
- [Stable-Baselines3](https://stable-baselines3.readthedocs.io/) — librería de referencia industrial para RL en Python