# Plan de implementación — Curva de aprendizaje + ε-decay

**Objetivo**: añadir dos mejoras al `practica_gymnasium.py` actual:
1. Sustituir el ruido gaussiano actual por **ε-greedy con decaimiento** (estándar industrial)
2. Añadir una **gráfica de convergencia** con `matplotlib` al final del entrenamiento

Tiempo estimado: **30–45 minutos**.

---

## Por qué merece la pena

La implementación actual usa ruido gaussiano (`np.random.randn * 1/(ep+1)`) como exploración. Funciona, pero tiene un problema: el ruido no es uniformemente distribuido entre acciones — favorece las que ya tienen Q-valor alto y distorsiona la selección. El **ε-greedy con decay** es la técnica estándar porque es transparente: durante `ε` fracción del tiempo explora uniformemente al azar, el resto explota la Q-table.

La **curva de convergencia** es lo que convierte un script que "llega a la meta" en uno que *demuestra aprendizaje* — visual e irrefutablemente.

---

## Paso 1 — Instalar matplotlib (si no está ya)

```bash
pip install matplotlib
```

Comprueba que funciona:
```bash
python3 -c "import matplotlib; print(matplotlib.__version__)"
```

---

## Paso 2 — Añadir el import de matplotlib

**Dónde**: al principio del script, junto al resto de imports (líneas 17-22 actuales).

```python
# Añadir después de "import numpy as np" (línea 22):
import matplotlib
matplotlib.use('Agg')       # Sin ventana gráfica interactiva (compatible con cualquier entorno)
import matplotlib.pyplot as plt
```

> `matplotlib.use('Agg')` es importante: evita que matplotlib abra una segunda ventana que choque con Pygame más adelante.

---

## Paso 3 — Añadir el parámetro epsilon a la sección de parámetros

**Dónde**: sección `# 3. PARÁMETROS` (líneas 55-57 actuales).

```python
# Añadir junto a eta, gamma, epis:
epsilon_inicio = 1.0    # Exploración total al principio (100% aleatoria)
epsilon_fin    = 0.01   # Exploración mínima al final (1% aleatoria)
# epsilon decae linealmente desde epsilon_inicio hasta epsilon_fin a lo largo de los episodios
```

---

## Paso 4 — Sustituir el bloque de selección de acción (el cambio principal)

**Dónde**: dentro del bucle `for episodio in range(epis)`, el bloque de selección de acción (líneas 79-83 actuales).

**Código actual** (líneas 79-83):
```python
# --- Selección de acción (epsilon-greedy con exploración decreciente) ---
# Al principio hay mucho ruido aleatorio (exploración).
# A medida que avanzan los episodios, el ruido disminuye (explotación).
ruido = np.random.randn(1, env_train.action_space.n) * (1.0 / (episodio + 1))
accion = int(np.argmax(Q[s, :] + ruido))
```

**Sustituir por**:
```python
# --- Selección de acción: ε-greedy con decaimiento lineal ---
# epsilon vale 1.0 en el episodio 0 (exploración total) y decae hasta 0.01
# en el último episodio (explotación casi total). Estándar en DQN y derivados.
epsilon = epsilon_fin + (epsilon_inicio - epsilon_fin) * (1 - episodio / epis)
if np.random.rand() < epsilon:
    accion = env_train.action_space.sample()   # Acción aleatoria uniforme (exploración)
else:
    accion = int(np.argmax(Q[s, :]))           # Mejor acción según Q-table (explotación)
```

> **Clave conceptual**: a diferencia del ruido gaussiano, `env_train.action_space.sample()` elige entre las 8 acciones con probabilidad exactamente uniforme — sin sesgo hacia ninguna.

---

## Paso 5 — Añadir la gráfica al final del entrenamiento

**Dónde**: después de `print(f"Entrenamiento completado...")` y antes de abrir el entorno demo (después de la línea 107 actual).

```python
# ---------------------------------------------------------------------------
# 4b. GRÁFICA DE CONVERGENCIA (curva de aprendizaje)
# ---------------------------------------------------------------------------

# Suavizamos la curva con una media móvil de ventana 50 para ver la tendencia real.
# Los valores crudos episodio a episodio son muy ruidosos al principio.
ventana  = 50
media_movil = np.convolve(
    recompensas_por_episodio,
    np.ones(ventana) / ventana,
    mode='valid'                # 'valid': solo devuelve valores donde la ventana está completa
)

fig, ax = plt.subplots(figsize=(10, 4))

# Recompensa cruda en gris claro (muestra la variabilidad episodio a episodio)
ax.plot(recompensas_por_episodio, color='lightsteelblue', alpha=0.4, label='Recompensa por episodio')

# Media móvil en azul oscuro (tendencia de convergencia)
ax.plot(
    range(ventana - 1, epis),   # Los primeros (ventana-1) episodios no tienen media completa
    media_movil,
    color='steelblue', linewidth=2, label=f'Media móvil (ventana={ventana})'
)

ax.set_xlabel('Episodio')
ax.set_ylabel('Recompensa acumulada')
ax.set_title(f'Curva de convergencia Q-Learning  |  η={eta}  γ={gamma}  ε: {epsilon_inicio}→{epsilon_fin}')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('curva_aprendizaje.png', dpi=150)   # Guardamos como imagen (no se abre ventana)
print(f"\nGráfica de convergencia guardada en 'curva_aprendizaje.png'")
```

---

## Paso 6 — Verificar que funciona

```bash
cd simuladores_ws_good/simuladores_gymnasium/
python3 practica_gymnasium.py
```

Deberías ver:
1. En consola: el progreso del entrenamiento igual que antes
2. Al final: `Gráfica de convergencia guardada en 'curva_aprendizaje.png'`
3. El fichero `curva_aprendizaje.png` en la misma carpeta
4. La ventana Pygame con la demo visual (igual que antes)

---

## Cómo leer la gráfica resultante

```
Recompensa
    0  |─────────────────────────────── episodio 500
       |       ░░░░░░░ (crudo, ruidoso)
  -50  |   ░░░░░░░                ─────── (media móvil: tendencia)
       |░░░      ──────────────────
 -100  |──────────
       |
 -150  |
```

- Si la **media móvil sube** (se acerca a 0) → el agente está aprendiendo ✅
- Si la curva **se estabiliza** antes del final → convergió, puedes reducir `epis`
- Si la curva **no mejora** → sube `epis` o ajusta `eta`/`gamma`

---

## Resumen de cambios

| Línea original | Cambio |
|---|---|
| 17-22 (imports) | Añadir `import matplotlib` + `matplotlib.use('Agg')` + `import matplotlib.pyplot as plt` |
| 55-57 (parámetros) | Añadir `epsilon_inicio = 1.0` y `epsilon_fin = 0.01` |
| 79-83 (selección acción) | Sustituir ruido gaussiano por ε-greedy con decaimiento lineal |
| Después de línea 107 | Insertar bloque completo de gráfica matplotlib |

**Total: ~20 líneas añadidas / 2 líneas modificadas.**
