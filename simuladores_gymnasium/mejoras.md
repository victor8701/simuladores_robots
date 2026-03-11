# Opciones de implementación — Práctica Gymnasium (Q-Learning y RL)

Guía de estudio sobre qué técnicas de aprendizaje por refuerzo se pueden implementar
en esta práctica, ordenadas por complejidad, con contexto de uso industrial real.

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

## Técnicas implementables (de menor a mayor complejidad)

### 1. Mejoras al Q-Learning actual

> **Uso industrial**: cualquier sistema con espacio de estados discreto y pequeño (robots industriales simples, videojuegos retro, trading de baja frecuencia).

Mejoras directas sobre el script actual sin cambiar el algoritmo:

| Mejora | Descripción | Dificultad |
|---|---|---|
| **Decaimiento de epsilon** | En lugar de ruido gaussiano, usar ε que decae de 1.0 a 0.01 a lo largo de los episodios | ⭐ |
| **Curva de aprendizaje** | Gráfica con `matplotlib` de recompensa media por episodio | ⭐ |
| **Guardar/cargar Q-Table** | `np.save` / `np.load` para reutilizar el entrenamiento | ⭐ |
| **Múltiples mapas** | Entrenar en varios CSV y comparar convergencia | ⭐⭐ |
| **Comparativa de hiperparámetros** | Barrer valores de `η` y `γ` y mostrar el efecto en la convergencia | ⭐⭐ |

---

### 2. SARSA (on-policy TD)

> **Uso industrial**: entornos donde el agente opera en producción durante el entrenamiento (robótica real, sistemas de recomendación on-line). Es más conservador que Q-Learning.

**Diferencia clave con Q-Learning:**

| | Q-Learning (off-policy) | SARSA (on-policy) |
|---|---|---|
| Actualización | `max Q(s')` | `Q(s', a')` (acción real tomada) |
| Política | Aprende la óptima aunque explore | Aprende la política que ejecuta |
| Riesgo | Puede sobreestimar valores | Más conservador, evita cliff-edges |

```python
# SARSA: elegir la siguiente acción ANTES de actualizar
a1 = elegir_accion(Q, s1, epsilon)
Q[s, a] += eta * (r + gamma * Q[s1, a1] - Q[s, a])
s, a = s1, a1
```

---

### 3. Q-Learning con replay buffer (DQN simplificado)

> **Uso industrial**: base de DeepMind's DQN (Atari, AlphaGo derivados). Se usa en robótica con espacio de estados grande.

En entornos pequeños como este no es necesario, pero se puede implementar para demostrar el concepto:

- Se guardan tuplas `(s, a, r, s')` en un buffer
- En cada paso se entrena con un mini-batch aleatorio del buffer
- Rompe la correlación temporal entre muestras consecutivas

---

### 4. Double Q-Learning

> **Uso industrial**: mejora estándar sobre DQN en aplicaciones financieras y de robótica. Reduce el sesgo de sobreestimación.

Mantiene dos Q-Tables (`Q_A` y `Q_B`). En cada paso se actualiza una aleatoriamente usando la otra para evaluar:

```python
# Double Q-Learning
if np.random.rand() < 0.5:
    a_best = np.argmax(Q_A[s1, :])
    Q_A[s, a] += eta * (r + gamma * Q_B[s1, a_best] - Q_A[s, a])
else:
    a_best = np.argmax(Q_B[s1, :])
    Q_B[s, a] += eta * (r + gamma * Q_A[s1, a_best] - Q_B[s, a])
```

---

### 5. Reward Shaping

> **Uso industrial**: crítico en robótica y navegación autónoma para acelerar el aprendizaje cuando la recompensa nativa es escasa.

Modificar la función de recompensa para que el agente reciba señales más informativas:

```python
# En lugar de solo r del entorno, añadir bonificación por acercarse a la meta
distancia_actual   = np.sqrt((x - goalX)**2 + (y - goalY)**2)
distancia_anterior = np.sqrt((x_prev - goalX)**2 + (y_prev - goalY)**2)
r_shaped = r + 0.1 * (distancia_anterior - distancia_actual)
```

---

### 6. Policy Gradient (REINFORCE) — avanzado

> **Uso industrial**: estándar actual en LLMs (RLHF), robótica continua (MuJoCo), juegos complejos. No es tabular: aprende directamente la política.

No aplica directamente al entorno CSV (espacio discreto pequeño), pero se puede implementar con una red neuronal mínima usando PyTorch:

```python
# La política es una red neuronal que mapea estado → probabilidad de cada acción
policy_net = torch.nn.Sequential(
    torch.nn.Linear(n_states, 64),
    torch.nn.ReLU(),
    torch.nn.Linear(64, n_actions),
    torch.nn.Softmax(dim=-1)
)
```

---

## Resumen de opciones recomendadas para la práctica

| Opción | Tipo de extra | Esfuerzo estimado | Impacto en nota |
|---|---|---|---|
| Curva de aprendizaje + métricas | Gymnasium | 30 min | Alto (muy visual) |
| Múltiples mapas + comparativa | Gymnasium | 1h | Alto |
| SARSA + comparativa con Q-Learning | Gymnasium | 1–2h | Muy alto |
| Decaimiento de epsilon + hiperparámetros | Gymnasium | 1h | Alto |
| `argparse` para parámetros por CLI | Programación general | 20 min | Medio |
| Guardar/cargar Q-Table | Programación general | 20 min | Medio |
| Double Q-Learning | Gymnasium | 1–2h | Alto |
| Reward Shaping | Gymnasium | 1h | Alto |

---

## Referencias

- Sutton & Barto — *Reinforcement Learning: An Introduction* (capítulos 6 y 7): referencia estándar de Q-Learning y SARSA
- [Gymnasium docs](https://gymnasium.farama.org/) — API oficial
- [DeepMind DQN paper (2015)](https://www.nature.com/articles/nature14236) — origen del replay buffer y Double DQN
- [OpenAI Spinning Up](https://spinningup.openai.com/) — guía práctica de algoritmos de RL modernos
