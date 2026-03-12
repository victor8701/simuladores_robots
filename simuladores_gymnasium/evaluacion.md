# Evaluación de la Práctica Gymnasium

## Rúbrica del enunciado

> **10% EC**: código funcional y comentado que mueva el robot origen→meta en al menos 1 mapa.
> **15% EC**: extras — cada extra relacionado con la API de Gymnasium suma hasta 7.5% EC; extras de programación general suman hasta 2.5% EC.

---

## ✅ 10% EC — Requisito base: CUBIERTO

- Entorno `gymnasium_csv-v0` con mapa CSV propio (`map1.csv`, 12×12 con obstáculos)
- Robot llega de (1,1) a (10,10) en **9 pasos** de forma consistente
- Código con **comentarios inline** en cada sección, parámetro y decisión de diseño

---

## ✅ Extras implementados

### Relacionados con la API de Gymnasium (hasta +7.5% EC cada uno)

| Extra | Detalle |
|-------|---------|
| **Q-Learning** | Tabla Q completa, regla de Bellman, episodios de entrenamiento |
| **ε-greedy con decay exponencial** | `ε × 0.9995` por episodio, de 1.0 → 0.01 |
| **Reward shaping** | `±0.1` por paso hacia/desde la meta — gradiente denso |
| **Curva de convergencia** | matplotlib: recompensa cruda + media móvil → `curva_aprendizaje.png` |

### Relacionados con programación general (hasta +2.5% EC cada uno)

| Extra | Detalle |
|-------|---------|
| **Animación suave** | Interpolación pygame de 20 frames/paso — sin saltos entre celdas |
| **Evasión visual de muros** | Movimiento en L cuando el diagonal cruza una esquina de pared |

---

## ❌ Posibles mejoras para subir nota

### Alta prioridad (Gymnasium API — +7.5% cada una)

| Mejora | Descripción |
|--------|-------------|
| **SARSA + comparativa** | Algoritmo on-policy; comparar curvas SARSA vs Q-Learning en el mismo mapa |
| **Segundo mapa** | Laberinto más estrecho que obligue a rodear obstáculos reales |
| **Double Q-Learning** | Dos tablas Q cruzadas para reducir sesgo de sobreestimación |

### Media prioridad (programación general — +2.5% cada una)

| Mejora | Descripción |
|--------|-------------|
| **Guardar/cargar Q-table** | `np.save` / `np.load` — demo sin reentrenar |
| **Heatmap de la Q-table** | Visualizar qué celdas tienen valores Q altos/bajos sobre el mapa |

---

## Convergencia verificada

```
Ep   500 / 10000:  -0.500   (nunca llega, exploración total)
Ep  2000 / 10000:  +0.100   (empieza a encontrar la meta)
Ep  5000 / 10000:  +0.840   (llega frecuentemente)
Ep 10000 / 10000:  +0.988   ← convergido ✓
```
