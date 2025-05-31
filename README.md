# p3_ekf_rme

Esta práctica consiste en la implementación de un filtro de Kalman extendido. 

Se han implementado tres versiones del filtro de Kalman extendido:
  1. EKF 3D: Este modelo solamente estima la posición y orientación del robot [x, y, θ].
  2. EKF 7D: Estimación de posición, velocidad y aceleración en línea recta [x, y, θ, v, ω, ax, ay]
  3. EKF 8D: Este modelo estima las dos componentes de la velocidad: [x, y, θ, vx, vy, ω, ax, ay].

Para la estimación con un EKF utilizaremos dos modelos matemáticos:
1. Modelo de movimiento (predicción): Formado por una matriz de transición A y una matriz de control B.
2. Modelo de observación (medición): Formado por una matriz de observación C

## EKF 3D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en `velocity_motion_models.py`, concretamente, `velocity_motion_model_linearized`. Mientras tanto, el modelo de observación será `odometry_observation_models.py`, más concretamente `odometry_observation_model_linearized`.

### Caso base
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.002, 0.002, 0.001] 
- Incertidumbre en observación: [1.02, 1.02, 100.01]

De acuerdo con la gráfica _Caso_base_3d.png_ que tenemos en la carpeta _Media_, podemos ver que la estimación no es muy buena, ya que la estimación del filtro no se ajusta a las líneas del gt


### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.002, 0.002, 0.001] 
- Incertidumbre en observación: [2.02, 2.02, 200.01]

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.005, 0.005, 0.003] 
- Incertidumbre en observación: [1.02, 1.02, 100.01]

## EKF 7D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en `velocity_motion_models.py`, concretamente, `velocity_motion_model_linearized`. Mientras tanto, el modelo de observación será `odometry_observation_models.py`, más concretamente `odometry_observation_model_linearized`.

### Caso base
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Caso_base_3d.png_ que tenemos en la carpeta _Media_, podemos ver que la estimación no es muy buena, ya que la estimación del filtro no se ajusta a las líneas del gt

### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [200.0, 200.0, 2000.0, 6.853891945200942e-03, 1.0966227112321507e-03, 0.003387262937311438, 0.003387262937311438]

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.5, 0.5, 0.25, 0.5, 0.5, 0.5, 0.5]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

## EKF 8D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en `velocity_motion_models.py`, concretamente, `velocity_motion_model_linearized`. Mientras tanto, el modelo de observación será `odometry_observation_models.py`, más concretamente `odometry_observation_model_linearized`.

### Caso base
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Caso_base_3d.png_ que tenemos en la carpeta _Media_, podemos ver que la estimación no es muy buena, ya que la estimación del filtro no se ajusta a las líneas del gt


### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [200.0, 200.0, 2000.0, 6.853891945200942e-03, 1.0966227112321507e-03, 0.003387262937311438, 0.003387262937311438]

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.5, 0.5, 0.25, 0.5, 0.5, 0.5, 0.5]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]
