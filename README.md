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

De acuerdo con la gráfica _Caso_base_3d.png_ que tenemos en la carpeta _Media_, podemos ver que la estimación no es muy buena, ya que la estimación del filtro no se ajusta a las líneas del ground truth. Tenemos una incertidumbre muy baja, pero la estimación del filtro no se superpone al valor real.

### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.002, 0.002, 0.001] 
- Incertidumbre en observación: [2.02, 2.02, 200.01]

De acuerdo con la gráfica _Ruido_alto_obs_3d.png_, podemos ver que la estimación ha empeorado respecto al caso anterior, ya que ahora ya no tenemos apenas corrección en la fase de actualización porque la incertidumbre de los sensores es muy alta, por lo que apenas se tienen en cuenta. Vemos que hay una gran diferencia entre las incertidumbres del modelo de movimiento y el de observación. 

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.01, 0.01, 0.005] 
- Incertidumbre en observación: [1.02, 1.02, 100.01]

De acuerdo con la gráfica _Ruido_alto_mov_3d.png_, podemos ver que la estimación ha mejorado respecto a los otros dos casos. Esto nos lleva a pensar que quizá el ruido del modelo de movimiento era excesivamente bajo y esto hacía que no se estuviese teniendo en cuenta la fase de actualización de la estimación del filtro. Vemos que ahora el filtro sí que se acerca más al ground truth, por lo que la estimación es mejor


## EKF 7D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en `acceleration_motion_models.py`, concretamente, `acceleration_motion_model_linearized_1`. Mientras tanto, el modelo de observación será `odometry_imu_observation_models.py`, más concretamente `odometry_imu_observation_model_with_acceleration_motion_model_linearized_1`.

### Caso base
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Caso_base_7d.png_ que tenemos en la carpeta _Media_, podemos ver que tenemos una gran incertidumbre, pero nos acercamos más al ground truth que en el caso anterior, por lo que se podría decir que este filtro funciona mejor. 

### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [200.0, 200.0, 2000.0, 6.853891945200942e-03, 1.0966227112321507e-03, 0.003387262937311438, 0.003387262937311438]

Como podemos ver en la gráfica _Ruido_alto_obs_7d.png_ de la carpeta _Media_, podemos ver que la estimación es peor que en el caso anterior, ya que tenemos una incertidumrbre muy alta en la medición de los sensores, por lo que apenas se tienen en cuenta en la fase de actualización y prácticamente funcionamos con solo la predicción. La incertidumbre es mayor que anteriormente. 

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.5, 0.5, 0.25, 0.5, 0.5, 0.5, 0.5]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Ruido_alto_mov_7d.png_, podemos ver que la estimación ha mejorado ligeramente respecto a los otros dos casos, por lo que podríamos decir que ahora el modelo funciona mejor, dándonos a pensar que los ruidos las configuraciones anteriores del filtro estaban un poco descompensados y quizá no eran realistas. 


## EKF 8D
Para esta implementación, el modelo de movimiento que utilizaremos será el que aparece en `acceleration_motion_models.py`, concretamente, `acceleration_motion_model_linearized_2`. Mientras tanto, el modelo de observación será `odometry_imu_observation_models.py`, más concretamente `odometry_imu_observation_model_with_acceleration_motion_model_linearized_2`.

### Caso base
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Caso_base_8d.png_ que tenemos en la carpeta _Media_, podemos ver que tenemos una menor incertidumbre que en el mismo caso del modelo 7D, pero la estimación no es tan buena. Aun así, funciona medianamente bien. 

### Incertidumbre alta en observación
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.1, 0.1, 0.05, 0.1, 0.1, 0.1, 0.1]
- Incertidumbre en observación: [200.0, 200.0, 2000.0, 6.853891945200942e-03, 1.0966227112321507e-03, 0.003387262937311438, 0.003387262937311438]

De acuerdo con la gráfica _Ruido_alto_obs_8d.png_, podemos ver que, al igual que en los casos de la misma configuración del caso 3D y 7D, tenemos una peor estimación porque no tenemos muy en cuenta la medición de los sensores y apenas se corrige la predicción que hemos hecho. 

### Incertidumbre alta en modelo de movimiento
Tendremos: 
- Incertidumbre en modelo de movimiento: [0.5, 0.5, 0.25, 0.5, 0.5, 0.5, 0.5]
- Incertidumbre en observación: [100.0, 100.0, 1000.0, 6.853891945200942e-06, 1.0966227112321507e-06, 0.0015387262937311438, 0.0015387262937311438]

De acuerdo con la gráfica _Ruido_alto_mov_8d.png_, podemos ver que, como en los mismos casos del modelo 3d y 7d, la estimación ha mejorado, por lo que, al igual que anteriormente, podríamos decir que los ruidos de observación y movimiento no eran realistas o no estaban compensados, por lo que solamente estábamos utilizando la fase de predicción del filtro de Kalman, obteniendo un resultado muy fiable. 
