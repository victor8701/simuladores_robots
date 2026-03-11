# Opciones de implementación — Práctica Gymnasium (Q-Learning y RL)

Guía de estudio sobre qué técnicas de aprendizaje por refuerzo se pueden implementar en esta práctica, ordenadas de MEJOR a PEOR opción según su valor en la robótica industrial real, su viabilidad en este entorno discreto y su impacto en la evaluación.

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

## Técnicas implementables (Ordenadas de MEJOR a PEOR opción para esta práctica)

### 1. Reward Shaping (La opción más recomendada)

> **Uso industrial**: Crítico en robótica y navegación autónoma para acelerar el aprendizaje cuando la recompensa nativa es escasa.
> **Por qué elegirla**: En la robótica industrial real, los robots rara vez reciben recompensas nativas constantes; los ingenieros deben "esculpir" estas señales. Para esta práctica, modificar la recompensa en base a la distancia a la meta es elegante, se implementa sobre tu entorno actual sin romper el algoritmo tabular, y tiene un alto impacto en la nota por solo 1 hora de esfuerzo estimado.

```python
# En lugar de solo r del entorno, añadir bonificación por acercarse a la meta
distancia_actual   = np.sqrt((x - goalX)**2 + (y - goalY)**2)
distancia_anterior = np.sqrt((x_prev - goalX)**2 + (y_prev - goalY)**2)
r_shaped = r + 0.1 * (distancia_anterior - distancia_actual)
```

---

### 2. SARSA (on-policy TD)

> **Uso industrial**: Entornos donde el agente opera en producción durante el entrenamiento (robótica real, sistemas de recomendación on-line). Es más conservador que Q-Learning y evita "cliff-edges".
> **Por qué elegirla**: Excelente si quieres entender cómo se entrenan robots que ya están operando en producción. A diferencia del Q-Learning que busca la política óptima asumiendo riesgos, SARSA aprende la política que realmente ejecuta. Su impacto en la nota es muy alto al requerir modificar el núcleo del algoritmo y compararlo.

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

### 3. Double Q-Learning

> **Uso industrial**: Mejora estándar sobre DQN en aplicaciones financieras y de robótica. Reduce el sesgo de sobreestimación.
> **Por qué elegirla**: En robótica avanzada, el sesgo de sobreestimación del Q-Learning clásico puede llevar a decisiones catastróficas. Double Q-Learning resuelve esto usando dos tablas Q cruzadas (`Q_A` y `Q_B`). Es una mejora estándar de la industria que encaja perfectamente en tu entorno discreto.

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

### 4. Mejoras operativas al Q-Learning actual

> **Uso industrial**: Cualquier sistema con espacio de estados discreto y pequeño (robots industriales simples, videojuegos retro, trading de baja frecuencia).
> **Por qué elegirlas**: Son modificaciones seguras y fundamentales para consolidar tu base y sumar puntos rápidos en la rúbrica (hasta 7.5% por extras de Gymnasium y 2.5% por extras de programación).

| Mejora | Descripción | Dificultad |
|---|---|---|
| **Decaimiento de epsilon** | En lugar de ruido gaussiano, usar ε que decae de 1.0 a 0.01 a lo largo de los episodios | ⭐ |
| **Curva de aprendizaje** | Gráfica con `matplotlib` de recompensa media por episodio | ⭐ |
| **Múltiples mapas** | Entrenar en varios CSV y comparar convergencia | ⭐⭐ |
| **Comparativa de hiperparámetros** | Barrer valores de `η` y `γ` y mostrar el efecto en la convergencia | ⭐⭐ |
| **Guardar/cargar Q-Table** | `np.save` / `np.load` para reutilizar el entrenamiento | ⭐ |

---

### 5. Q-Learning con replay buffer (DQN simplificado)

> **Uso industrial**: Base de DeepMind's DQN (Atari, AlphaGo derivados). Se usa en robótica con espacio de estados grande.
> **Por qué relegarla**: Aunque es un concepto fundacional clave, para un mapa pequeño en CSV es una sobreingeniería innecesaria. Guarda tuplas `(s, a, r, s')` en un buffer para romper la correlación temporal, pero el esfuerzo de implementación no compensa el aprendizaje práctico en tu entorno actual en comparación con Reward Shaping o SARSA.

---

### 6. Policy Gradient (REINFORCE) — La peor opción para este entorno

> **Uso industrial**: Estándar actual en LLMs (RLHF), robótica continua (MuJoCo), juegos complejos. 
> **Por qué relegarla**: Paradójicamente, aunque es el estándar actual absoluto en la industria, es la peor opción para esta práctica específica. No es un método tabular; aprende mapeando estados a probabilidades mediante redes neuronales, lo cual no aplica directamente a tu entorno CSV sin introducir frameworks complejos (como PyTorch) que se escapan del objetivo de la práctica.

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
| Reward Shaping | Gymnasium | 1h | Alto |
| SARSA + comparativa con Q-Learning | Gymnasium | 1–2h | Muy alto |
| Double Q-Learning | Gymnasium | 1–2h | Alto |
| Curva de aprendizaje + métricas | Gymnasium | 30 min | Alto (muy visual) |
| Múltiples mapas + comparativa | Gymnasium | 1h | Alto |
| Decaimiento de epsilon + hiperparámetros | Gymnasium | 1h | Alto |
| Guardar/cargar Q-Table | Programación general | 20 min | Medio |
| `argparse` para parámetros por CLI | Programación general | 20 min | Medio |

---

## Referencias

- Sutton & Barto — *Reinforcement Learning: An Introduction* (capítulos 6 y 7): referencia estándar de Q-Learning y SARSA
- [Gymnasium docs](https://gymnasium.farama.org/) — API oficial
- [DeepMind DQN paper (2015)](https://www.nature.com/articles/nature14236) — origen del replay buffer y Double DQN
- [OpenAI Spinning Up](https://spinningup.openai.com/) — guía práctica de algoritmos de RL modernos