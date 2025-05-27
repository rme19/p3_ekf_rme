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
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en _odometry_motion_models.py_, mientras que el modelo de observación será _velocity_motion_models.py_



## EKF 7D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en _odometry_motion_models.py_, mientras que el modelo de observación será _velocity_motion_models.py_



## EKF 8D

