# Robot Labyrinth Navigation with Gymnasium Q-Learning

Este subproyecto entrena un agente Inteligente artificial mediante Aprendizaje por Refuerzo (Reinforcement Learning) en Python, solucionando el problema del "Laberinto" en mapas grid bidimensionales proporcionados por CSV.

## Descripción General
Utiliza la API moderna **OpenAI Gymnasium** y NumPy para definir y resolver los entornos. El script `practica_gymnasium.py` instancia un mapeado CSV en pantalla y entrena a un agente para que viaje con éxito desde la celda de origen a la celda meta castigando cualquier choque contra paredes. Al finalizar se muestra visualmente usando PyGame la ruta óptima aprendida y se renderiza un gráfico de la convergencia de la recompensa para verificar la fiabilidad del entrenamiento.

## Algoritmo Utilizado
El núcleo es el algoritmo **Q-Learning Tabular**. 
Debido a la escasez natural de puntos de recompensa ("Sparse Rewards") del laberinto (sólo se cobra al pisar la meta final), aplicamos **Potencial-Based Reward Shaping**:
- Se evalúa la distancia verdadera hacia el objetivo usando Búsqueda en Anchura (**BFS**) antes de comenzar el entrenamiento.
- En cada paso del agente guiamos su conocimiento premiando activamente los desplazamientos que reduzcan esta distancia BFS, acelerando inmensamente el entrenamiento y previniendo que la máquina caiga en "óptimos locales" (bucles oscilantes cerca de paredes que aparentan llevar hacia la meta en distancia lineal/Manhattan pero se hallan realmente tapiadas).

## Archivos y Mapas

- `practica_gymnasium.py`: Core del código de Q-Learning.
- `map1.csv` a `map5.csv`: Diferentes mapas laberínticos discretizados ascendentes en dificultad y tamaño. Map1 es de `10x10`, Map5 es de gigante de `50x50`.
- Imágenes `.png`: Contienen evidencias visuales del progreso del agente al a lo largo de los distintos mapas laberinto (`curva_aprendizaje_mapaX.png`).
- Documentos Técnicos: Detalles de mejoras de diseño y plan de trabajo (`mejoras.md`, `evaluacion.md`).

## Requisitos de Sistema e Instalación
- Python 3.10+
- Bibliotecas PIP necesarias:
  * `gymnasium`
  * `numpy`
  * `pygame`
  * `matplotlib`

Además, se requiere clocar internamente el paquete `gymnasium-csv`.

## Ejecución Básica
Para arrancar el entrenamiento sobre algún mapa en específico, utilice el parámetro `--mapa`.
Ejemplo para resolver y ver gráficamente el gran laberinto 5 (50x50):
```bash
python3 practica_gymnasium.py --mapa 5
```
