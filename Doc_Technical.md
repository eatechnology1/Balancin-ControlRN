# 📚 Documentación Técnica Completa – Balancín-ControlRN

## 1. Visión general del sistema

El proyecto **Balancín-ControlRN** implementa el control de un **robot balancín seguidor de línea** sobre ESP32-S3, combinando:

- Medición de ángulo con **MPU6050**.
- Estimación de ángulo mediante **filtro complementario**.
- Medición de velocidad de ruedas con **encoders incrementales**.
- Control **en cascada** (ángulo → velocidad → PWM).
- Una **red neuronal feed-forward** que ajusta dinámicamente la ganancia proporcional del lazo de ángulo.
- Un **PID de línea** que genera un factor de giro multiplicativo para seguir una línea negra con 8 sensores.
- Arquitectura basada en **FreeRTOS** con dos tareas principales (balanceo y línea).

El objetivo es mantener el robot vertical y, al mismo tiempo, seguir una línea en el suelo de forma estable y suave.

---

## 2. Arquitectura de software 🧱

### 2.1. Módulos principales

- `main.cpp`  
  - Inicializa Serial, I2C, MPU6050, PWM, encoders, red neuronal, PID de línea y tareas FreeRTOS.
  - Crea las tareas `TaskBalanceo` y `TaskLinea`.
  - Gestiona el botón de calibración de la MPU.

- `config.h`  
  - Define pines de motores, encoders, MPU, botón, sensores de línea.
  - Parámetros globales: frecuencia de PWM, resolución, CPR de encoders, número de sensores, etc.

- `encoders.h`  
  - Variables `volatile long countL, countR`.
  - ISRs `isr_LA`, `isr_RA` que actualizan los contadores según la fase A/B del encoder.
  - Funciones auxiliares en el código principal calculan RPM a partir de los conteos y el tiempo.

- `motors.h`  
  - Configuración de timers y canales PWM de LEDC (modo low-speed).
  - Función `driveMotorsDifferential(pwmL, pwmR)`:
    - Determina dirección (adelante/atrás) según el signo.
    - Aplica `constrain` de \(|PWM|\) a \([0, 255]\).
    - Actualiza el duty de cada canal.

- `mpu_block.h`  
  - Objeto global `MPU6050 mpu`.
  - Gestión de calibración con `Preferences` (NVS).
  - Cálculo de ángulo a partir de acelerómetro y giroscopio.
  - Implementación del **filtro complementario** para obtener un ángulo filtrado `angleFiltered`.

- `nn_cascade_block.h`  
  - Configuración de la **red neuronal feed-forward** (estructura, funciones de activación, pesos).
  - Estructuras de datos dinámicas (Dynamic_Array).
  - Función `initNeural()` para inicializar pesos, escalas y tipos de activación.
  - Función `cascada(...)` que implementa:
    - Lazo de ángulo (PID cuya `Kp` se ajusta con la RN).
    - Lazo de velocidad (PI).
    - Integración de errores y saturación de salidas.

- `line_follower_block.h`  
  - Definición del PID de línea.
  - Lectura de 8 sensores digitales.
  - Cálculo de posición de la línea.
  - Cálculo de `outputPIDLinea` y conversión a `factor_giro` (rango típico aprox. [-0.4, 0.4]).

- `tasks_block.h`  
  - Función `TaskBalanceo(void* pvParameters)`.
  - Función `TaskLinea(void* pvParameters)`.
  - `stopRobotAndTasks()` / `resumeRobotAndTasks()` para pausar/reanudar tareas durante la calibración.
  - Creación de tareas con `xTaskCreatePinnedToCore`.

---

## 3. Modelado del robot balancín 📐

> Nota: El firmware no resuelve explícitamente el modelo matemático completo del péndulo invertido, pero el diseño del cascada y la RN están inspirados en ese comportamiento.

### 3.1. Variables principales

- \( \theta \): ángulo del robot respecto a la vertical (rad o grados).
- \( \dot{\theta} \): velocidad angular (derivada del giroscopio).
- \( v_L, v_R \): velocidades de las ruedas izquierda y derecha (RPM).
- \( v \): velocidad lineal aproximada del robot (media de ambas ruedas).
- `angleFiltered`: estimación de \( \theta \) tras el filtro complementario.
- `rpmL_f`, `rpmR_f`: velocidades filtradas.

### 3.2. Encoders → RPM

Suponiendo:

- `CPR`: cuentas por revolución por canal.
- \( \Delta N_L, \Delta N_R \): incremento de cuenta en un período \( \Delta t \).

RPM de cada rueda:

- \( \text{RPM}_L = \dfrac{\Delta N_L}{\text{CPR}} \cdot \dfrac{60}{\Delta t} \)
- \( \text{RPM}_R = \dfrac{\Delta N_R}{\text{CPR}} \cdot \dfrac{60}{\Delta t} \)

Velocidad media:

- \( v = \dfrac{\text{RPM}_L + \text{RPM}_R}{2} \)

Filtro IIR (pasa‑bajas) para las RPM:

- \( \text{RPM}_{\text{filtrada}}(k) = \alpha \cdot \text{RPM}_{\text{filtrada}}(k-1) + (1 - \alpha) \cdot \text{RPM}_{\text{medida}}(k) \)

con \( \alpha \) cercano a 1.

---

## 4. Estimación de ángulo con MPU6050 🎛️

### 4.1. Ángulo por acelerómetro

Con \( a_x, a_y, a_z \) en unidades de \( g \):

- \( \theta_{\text{acc}} = \arctan2(a_x, a_z) \)

Es una medida buena a baja frecuencia, pero ruidosa.

### 4.2. Ángulo por giroscopio

El giroscopio entrega velocidad angular \( \omega \) (por ejemplo grados/s). Integrando:

- \( \theta_{\text{gyro}}(k) = \theta_{\text{gyro}}(k-1) + \omega(k) \cdot \Delta t \)

Buena a alta frecuencia, pero con deriva.

### 4.3. Filtro complementario

Combinación de ambas:

- \( \theta_{\text{filt}}(k) = \alpha \left[ \theta_{\text{filt}}(k-1) + \omega(k) \cdot \Delta t \right] + (1 - \alpha)\, \theta_{\text{acc}}(k) \)

Con \( \alpha \in (0,1) \), usualmente entre 0.90 y 0.99.

---

## 5. Control en cascada ⚙️

Dos lazos:

1. **Lazo de ángulo (externo)**  
   - Medida: `angleFiltered`.  
   - Referencia: \( \theta_{\text{ref}} = 0 \) (vertical).  
   - Salida: referencia de velocidad o corrección.

2. **Lazo de velocidad (interno)**  
   - Medida: velocidad (RPM media).  
   - Referencia: salida del lazo de ángulo.  
   - Salida: PWM base que va a los motores.

### 5.1. PID de ángulo

Error:

- \( e_\theta(k) = \theta_{\text{ref}}(k) - \theta_{\text{filt}}(k) \)

PID:

- \( u_\theta(k) = K_p^\theta e_\theta(k) + K_i^\theta \sum_{i=0}^{k} e_\theta(i)\Delta t + K_d^\theta \dfrac{e_\theta(k) - e_\theta(k-1)}{\Delta t} \)

En tu sistema:

- \( K_p^\theta \) es ajustado por la red neuronal.
- El resultado se usa como referencia de velocidad o aporte al PWM interno.

### 5.2. PI de velocidad

Error:

- \( e_v(k) = v_{\text{ref}}(k) - v(k) \)

PI:

- \( u_v(k) = K_p^v e_v(k) + K_i^v \sum_{i=0}^{k} e_v(i)\Delta t \)

`u_v` es el PWM base (antes de aplicar el factor de giro) y luego se satura a \([-PWM_{\max}, PWM_{\max}]\).

---

## 6. Red neuronal feed-forward 🧠

### 6.1. Estructura general

La RN es un perceptrón multicapa feed‑forward:

- \( \text{Input} \rightarrow \text{Hidden layers} \rightarrow \text{Output} \)

Entradas típicas:

- Error de ángulo \( e_\theta \).
- Error de velocidad \( e_v \).
- Quizá derivadas o valores previos.

Capas ocultas:

- Neuronas con activación `logsig` (sigmoide logística).

Salida:

- Escalar que modula \( K_p^\theta \) o actúa como factor multiplicativo.

Salida general:

- \( y = f_{\text{out}}\left( W^{(L)} f_{L-1}(\dots f_1( W^{(1)} x + b^{(1)} ) \dots ) + b^{(L)} \right) \)

### 6.2. Funciones de activación

- `logsig` (sigmoide logística):  
  - \( \text{logsig}(z) = \dfrac{1}{1 + e^{-z}} \)

- `poslin_lim` (posible lineal positiva limitada):  
  - \( \text{poslin\_lim}(z) = 0 \) si \( z < 0 \)  
  - \( \text{poslin\_lim}(z) = z \) si \( 0 \le z \le z_{\max} \)  
  - \( \text{poslin\_lim}(z) = z_{\max} \) si \( z > z_{\max} \)

La combinación da salidas suaves y acotadas.

### 6.3. Rol en el control

Idea:

1. El PID clásico genera un comportamiento base.
2. La RN observa errores y estados.
3. Genera un ajuste de \( K_p^\theta \):

   - \( K_{p,\text{ef}}^\theta = K_{p,\text{base}}^\theta + \Delta K_p^\theta \)  
     o  
   - \( K_{p,\text{ef}}^\theta = K_{p,\text{base}}^\theta \cdot (1 + y) \)

4. Esto compensa cambios de masa, fricción, montaje, etc.

La evaluación y aplicación se hace dentro de `cascada(...)` en cada ciclo de control.

---

## 7. Seguidor de línea 🧵

### 7.1. Sensores y codificación

- 8 sensores digitales \( S_0, \dots, S_7 \), cada uno vale 0 (negro) o 1 (blanco).
- Posición de la línea:

  - \( \text{pos} = \dfrac{\sum_{i=0}^{7} w_i s_i}{\sum_{i=0}^{7} s_i} \)

  donde:

  - \( w_i \) son las posiciones 0, 100, 200, …, 700.
  - \( s_i \) son 0/1.

Rango típico: 0–700, centro ≈ 350.

### 7.2. PID de línea

Error de línea:

- \( e_{\text{line}}(k) = \text{pos}_{\text{ref}} - \text{pos}(k) \)

PID:

- \( u_{\text{line}}(k) = K_p^{\text{line}} e_{\text{line}}(k) + K_i^{\text{line}} \sum e_{\text{line}}(i)\Delta t + K_d^{\text{line}} \dfrac{e_{\text{line}}(k) - e_{\text{line}}(k-1)}{\Delta t} \)

Se escala para obtener `factor_giro`:

- \( \text{factor\_giro} = \text{sat}\left( \dfrac{u_{\text{line}}}{\text{escala}} \right) \), con \(\text{sat}(\cdot) \in [-f_{\max}, f_{\max}]\).

### 7.3. Aplicación al PWM

Si `pwmBase` es el PWM del cascada:

- \( pwm_L = pwmBase \cdot (1 + \text{factor\_giro}) \)
- \( pwm_R = pwmBase \cdot (1 - \text{factor\_giro}) \)

- `factor_giro > 0`: rueda izquierda acelera, derecha frena → giro a la derecha.
- `factor_giro < 0`: al revés → giro a la izquierda.

---

## 8. Tareas FreeRTOS y flujo 🧵⏱️

### 8.1. TaskBalanceo

Período típico: 20 ms (50 Hz).

Pseudoflujo:

1. Medir \( \Delta t \).
2. Leer MPU6050 (acc + gyro).
3. Actualizar `angleFiltered` con el filtro complementario.
4. Leer y resetear contadores de encoders.
5. Calcular RPM y aplicar filtro IIR.
6. Llamar a `cascada(angleFiltered, rpmL_f, rpmR_f, dt)`:
   - Actualizar errores.
   - Ejecutar RN.
   - Calcular PWM base saturado.
7. Combinar con `factor_giro` → `pwmL`, `pwmR`.
8. `driveMotorsDifferential(pwmL, pwmR)`.

### 8.2. TaskLinea

Período típico: 100 ms (10 Hz).

Pseudoflujo:

1. Leer los 8 sensores.
2. Calcular posición de la línea.
3. Calcular error.
4. Ejecutar PID de línea.
5. Actualizar `factor_giro`.

### 8.3. Calibración de MPU

1. Detectar pulsación de `BTN_CAL`.
2. `stopRobotAndTasks()`:
   - PWM = 0.
   - Suspender tareas de balanceo y línea.
3. `calibrateMPU()`:
   - Tomar N muestras.
   - Calcular offsets.
   - Guardar en NVS.
4. Recalcular ángulo inicial.
5. `resumeRobotAndTasks()`.

---

## 9. Flujo de trabajo del desarrollador 👨‍💻

- Rama principal: `main`.

Flujo típico:

```
git status
git add .
git commit -m "Descripción del cambio"
git push
```

En caso de cambios remotos:

```
git pull
```

Compilación y carga con PlatformIO:

```
pio run -t upload
pio device monitor -b 115200
```

---

## 10. Ideas de mejora 🌱

- Modo de solo balanceo (sin seguidor de línea).
- Interfaz serie/web para ajustar parámetros PID y de la RN en tiempo real.
- Registro de telemetría para análisis offline.
- Experimentar con RNN o reinforcement learning para swing‑up + balanceo.

---

## 11. Resumen conceptual 🧠

- El robot es un **péndulo invertido sobre ruedas**.
- El lazo de ángulo mantiene la “vara” vertical.
- El lazo de velocidad traduce esa corrección en movimiento de ruedas.
- La red neuronal ajusta parámetros del controlador para adaptarse a cambios.
- El PID de línea corrige suavemente la trayectoria sin romper el equilibrio.
- FreeRTOS separa la lógica de balanceo rápido del seguimiento de línea más lento.