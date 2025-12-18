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
  - Funciones auxiliares (en tu código principal) calculan RPM a partir de los conteos y el tiempo.

- `motors.h`  
  - Configuración de timers y canales PWM de **LEDC** (modo low-speed).
  - Función `driveMotorsDifferential(pwmL, pwmR)`:
    - Determina dirección (adelante/atrás) según el signo.
    - Hace `constrain` de |PWM| a [0, 255].
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

- \( \theta \): ángulo del robot respecto a la vertical (rad o grados, en el código se maneja en grados y se escala a rad solo cuando hace falta).
- \( \dot{\theta} \): velocidad angular (derivada del giroscopio).
- \( v_L, v_R \): velocidades de las ruedas izquierda y derecha (RPM).
- \( v \): velocidad lineal aproximada del robot (media de ambas ruedas).
- `angleFiltered`: estimación de \(\theta\) tras el filtro complementario.
- `rpmL_f`, `rpmR_f`: velocidades filtradas.

### 3.2. Encoders → RPM

Suponiendo:

- `CPR`: cuentas por revolución por canal (puede multiplicarse por 4 si se usan flancos de A y B).
- \( \Delta N_L, \Delta N_R \): incremento de cuenta en un período \(\Delta t\).

La RPM de cada rueda se aproxima como:

\[
\text{RPM}_L = \frac{\Delta N_L}{\text{CPR}} \cdot \frac{60}{\Delta t}
\]

\[
\text{RPM}_R = \frac{\Delta N_R}{\text{CPR}} \cdot \frac{60}{\Delta t}
\]

La velocidad media:

\[
v = \frac{\text{RPM}_L + \text{RPM}_R}{2}
\]

Tu código aplica además un **filtro IIR** (pasa-bajas) a las RPM:

\[
\text{RPM}_{\text{filtrada}}(k) = \alpha \cdot \text{RPM}_{\text{filtrada}}(k-1) + (1 - \alpha) \cdot \text{RPM}_{\text{medida}}(k)
\]

con \( \alpha \) cercano a 1 para suavizar.

---

## 4. Estimación de ángulo con MPU6050 🎛️

### 4.1. Ángulo por acelerómetro

El acelerómetro entrega \(a_x, a_y, a_z\) en unidades de \(g\) (tras aplicar factores de escala). El ángulo de inclinación (por ejemplo pitch) se puede aproximar como:

\[
\theta_{\text{acc}} = \arctan2(a_x, a_z)
\]

o alguna variante según el eje elegido.

Este ángulo es bueno a baja frecuencia (baja dinámica), pero ruidoso.

### 4.2. Ángulo por giroscopio

El giroscopio entrega la velocidad angular \(\omega\) (por ejemplo en grados/s). Integrando en el tiempo:

\[
\theta_{\text{gyro}}(k) = \theta_{\text{gyro}}(k-1) + \omega(k) \cdot \Delta t
\]

Este ángulo es bueno a alta frecuencia, pero sufre **deriva** con el tiempo.

### 4.3. Filtro complementario

El filtro complementario combina ambas estimaciones:

\[
\theta_{\text{filt}}(k) = \alpha \left[ \theta_{\text{filt}}(k-1) + \omega(k) \cdot \Delta t \right] + (1 - \alpha)\, \theta_{\text{acc}}(k)
\]

- \( \alpha \in (0,1) \) (típicamente entre 0.90 y 0.99). [web:99][web:102]
- Alto peso a la integración del giroscopio (respuesta rápida).
- Bajo peso al ángulo del acelerómetro (corrige deriva en el largo plazo).

En tu módulo `mpu_block`, esta lógica se implementa dentro de la función que actualiza `angleFiltered`, usando el tiempo entre lecturas como \(\Delta t\).

---

## 5. Control en cascada ⚙️

El controlador principal está organizado en **dos lazos**:

1. **Lazo de ángulo** (externo):  
   - Variable medida: `angleFiltered`.
   - Referencia: \( \theta_{\text{ref}} = 0 \) (robot vertical).
   - Salida: referencia de velocidad o término “corrección” para la velocidad.

2. **Lazo de velocidad** (interno):  
   - Variable medida: velocidad (RPM media de las ruedas).
   - Referencia: salida del lazo de ángulo (y posiblemente otros términos).
   - Salida: PWM base que va a los motores.

### 5.1. PID de ángulo

Error de ángulo:

\[
e_\theta(k) = \theta_{\text{ref}}(k) - \theta_{\text{filt}}(k)
\]

Controlador PID:

\[
u_\theta(k) = K_p^\theta \, e_\theta(k) + K_i^\theta \sum_{i=0}^{k} e_\theta(i)\Delta t + K_d^\theta \frac{e_\theta(k) - e_\theta(k-1)}{\Delta t}
\]

En tu sistema:

- \( K_p^\theta \) no es fijo: la **red neuronal** proporciona un factor para adaptarlo.
- El resultado se convierte en una referencia de velocidad o directamente en un aporte al PWM interno.

### 5.2. PI de velocidad

Error de velocidad:

\[
e_v(k) = v_{\text{ref}}(k) - v(k)
\]

Controlador PI:

\[
u_v(k) = K_p^v \, e_v(k) + K_i^v \sum_{i=0}^{k} e_v(i)\Delta t
\]

`u_v` es el **PWM base** (antes de aplicar el factor de giro del seguidor de línea). Luego se saturará a \([-PWM_{\max}, PWM_{\max}]\).

---

## 6. Red neuronal feed-forward 🧠

### 6.1. Estructura general

La RN implementada es un **perceptrón multicapa feed-forward**:

\[
\text{Input} \rightarrow \text{Hidden layers} \rightarrow \text{Output}
\]

En tu caso (según el módulo `nn_cascade_block`):

- Entrada: vector que incluye, por ejemplo:
  - Error de ángulo \( e_\theta \).
  - Error de velocidad \( e_v \).
  - Derivadas o valores previos (según cómo se definieron las características).
- Capas ocultas: al menos 1 capa con neuronas tipo **logsig** (sigmoide logística saturada).
- Salida: una variable escalar que modula \( K_p^\theta \) (o directamente un factor multiplicativo sobre la salida del PID).

La salida de la RN se puede escribir como:

\[
y = f_{\text{out}}\left( W^{(L)} \cdot f_{L-1} \left( \dots f_1\left( W^{(1)} x + b^{(1)} \right) \dots \right) + b^{(L)} \right)
\]

donde:

- \(x\): vector de entrada.
- \(W^{(l)}, b^{(l)}\): pesos y biases de la capa \(l\).
- \(f_l(\cdot)\): función de activación (por ejemplo **logsig**).
- \(y\): salida escalar usada para modificar el controlador.

### 6.2. Funciones de activación

Has configurado la RN con strings como `"logsig"` y `"poslin_lim"`:

- `logsig`: sigmoide logística

\[
\text{logsig}(z) = \frac{1}{1 + e^{-z}}
\]

- `poslin_lim`: posiblemente una versión limitada de ReLU o “positiva lineal limitada”, por ejemplo:

\[
\text{poslin\_lim}(z) =
\begin{cases}
0, & z < 0 \\
z, & 0 \le z \le z_{\max} \\
z_{\max}, & z > z_{\max}
\end{cases}
\]

La combinación de estas funciones da un comportamiento **suave** y al mismo tiempo **acotado** para la salida, evitando que los ajustes de \(K_p^\theta\) sean extremos.

### 6.3. Rol de la RN en el control

La idea es:

1. El sistema (PID clásico) genera un comportamiento base de balanceo.
2. La RN observa **errores** y/o variables de estado.
3. La RN genera un factor \( \Delta K_p^\theta \) o un factor multiplicativo:

\[
K_{p,\text{ef}}^\theta = K_{p,\text{base}}^\theta + \Delta K_p^\theta
\quad \text{o} \quad
K_{p,\text{ef}}^\theta = K_{p,\text{base}}^\theta \cdot (1 + y)
\]

4. Esto permite adaptarse a cambios en:
   - Masa del robot (batería, carga).
   - Coeficientes de fricción.
   - Pequeñas variaciones en el montaje mecánico.

En tu código, la lógica de actualización se encapsula en `cascada(...)`: la RN se evalúa cada ciclo, y la salida se usa para actualizar internamente la acción de control.

---

## 7. Seguidor de línea 🧵

### 7.1. Sensores y codificación

- 8 sensores digitales (por ejemplo \(S_0, \dots, S_7\)), cada uno devuelve `0` (negro) o `1` (blanco).
- La posición de la línea se calcula asignando un peso a cada sensor:

\[
\text{pos} = \frac{\sum_{i=0}^{7} w_i \cdot s_i}{\sum_{i=0}^{7} s_i}
\]

donde:

- \(w_i\) son posiciones nominales (0, 100, 200, …, 700).
- \(s_i\) son 0/1 (o invertidos según hardware).

El rango suele ser 0–700 y el centro (línea bajo el centro del robot) se ubica cerca de 350.

### 7.2. PID de línea

Se define un error de línea:

\[
e_{\text{line}}(k) = \text{pos}_{\text{ref}} - \text{pos}(k)
\]

Normalmente \(\text{pos}_{\text{ref}} = 350\) ó el valor que corresponda al centro.

PID:

\[
u_{\text{line}}(k) = K_p^{\text{line}} e_{\text{line}}(k) + K_i^{\text{line}} \sum e_{\text{line}}(i)\Delta t + K_d^{\text{line}} \frac{e_{\text{line}}(k) - e_{\text{line}}(k-1)}{\Delta t}
\]

Luego se escala y se satura para generar `factor_giro`:

\[
\text{factor\_giro} = \text{sat}\left( \frac{u_{\text{line}}}{\text{escala}} \right), \quad \text{con } \text{sat}(\cdot) \in [-f_{\max}, f_{\max}]
\]

### 7.3. Aplicación al PWM de motores

Si `pwmBase` es el PWM salido del cascada (balanceo), entonces:

\[
pwm_L = pwmBase \cdot (1 + \text{factor\_giro})
\]

\[
pwm_R = pwmBase \cdot (1 - \text{factor\_giro})
\]

- Si `factor_giro > 0`: la rueda izquierda acelera y la derecha frena → giro a la derecha.
- Si `factor_giro < 0`: al revés → giro a la izquierda.

De esta forma, el **balanceo** se mantiene y el robot corrige su dirección para seguir la línea.

---

## 8. Tareas FreeRTOS y flujo de ejecución 🧵⏱️

### 8.1. TaskBalanceo

Período típico: ~20 ms (50 Hz), usando `vTaskDelay` o `xTaskDelayUntil`.

Pseudoflujo:

1. Medir tiempo \(\Delta t\).
2. Leer MPU6050 (acc + gyro).
3. Actualizar `angleFiltered` mediante el filtro complementario.
4. Leer y resetear contadores de encoders.
5. Calcular RPM y aplicar filtro IIR.
6. Llamar a `cascada(angleFiltered, rpmL_f, rpmR_f, dt)`:
   - Actualiza errores.
   - Ejecuta RN para ajustar parámetros.
   - Calcula PWM base saturado.
7. Combinar con `factor_giro`:
   - `pwmL`, `pwmR`.
8. Llamar a `driveMotorsDifferential(pwmL, pwmR)`.

### 8.2. TaskLinea

Período típico: ~100 ms (10 Hz).

Pseudoflujo:

1. Leer los 8 sensores de línea.
2. Calcular posición de la línea.
3. Calcular error de línea.
4. Ejecutar PID de línea.
5. Actualizar `factor_giro` (global, protegida si hace falta).

### 8.3. Calibración de MPU (botón)

En `loop` o en una tarea auxiliar:

1. Detectar pulsación del botón `BTN_CAL`.
2. Llamar a `stopRobotAndTasks()`:
   - Poner PWM a 0.
   - Suspender tareas de balanceo y línea.
3. Llamar a `calibrateMPU()`:
   - Tomar `N` muestras.
   - Calcular offsets promedio.
   - Ajustar `ax_offset`, `ay_offset`, `az_offset`, `gx_offset`.
   - Guardar en `Preferences`.
4. Recalcular ángulo inicial.
5. Llamar a `resumeRobotAndTasks()`.

---

## 9. Flujo de trabajo del desarrollador 👨‍💻

### 9.1. Git y versionado

- Rama principal: `main`.
- Flujo típico:

```
# Ver estado
git status

# Añadir cambios
git add .

# Commit
git commit -m "Descripción del cambio"

# Subir a GitHub
git push
```

- En caso de cambios remotos:

```
git pull
```

### 9.2. Compilación y carga (PlatformIO)

```
pio run -t upload           # Compilar y subir firmware
pio device monitor -b 115200  # Monitor serie
```

---

## 10. Ideas de mejora / extensiones 🌱

- Añadir un **modo de solo balanceo** (sin seguidor de línea) seleccionable por botón.
- Implementar una **interfaz serie o web** para ajustar parámetros PID y de la RN en tiempo real.
- Registrar datos de telemetría (ángulo, RPM, salida RN) para análisis offline.
- Explorar otras técnicas de IA:
  - Redes recurrentes (RNN/LSTM) para ver si mejoran la anticipación de caídas.
  - Control por **reinforcement learning** para swing-up + balanceo. [web:110][web:116]

---

## 11. Resumen conceptual 🧠

- El robot es un **péndulo invertido sobre ruedas**.
- El lazo de ángulo trata de “mantener la vara vertical”.
- El lazo de velocidad traduce esa corrección en movimiento de las ruedas.
- La **red neuronal** ajusta parámetros del controlador para adaptarlo a cambios en el sistema.
- El **PID de línea** desvía ligeramente el par de motores para seguir una línea sin romper el equilibrio.
- FreeRTOS permite separar la lógica de **balanceo rápido** de la lógica de **seguimiento de línea más lenta**.

---
```

[1](https://forum.arduino.cc/t/mpu6050-complementary-filter/315523)
[2](https://forum.arduino.cc/t/mpu6050-with-arduino-complementary-filter/388384)
[3](https://www.reddit.com/r/arduino/comments/5syouj/mpu6050_complementary_filter/)
[4](https://www.hibit.dev/posts/92/complementary-filter-and-relative-orientation-with-mpu6050)
[5](https://www.youtube.com/watch?v=OTuk-GdoPUQ)
[6](https://onlinelibrary.wiley.com/doi/10.1155/2021/5536573)
[7](https://www.reddit.com/r/esp32/comments/1m5ks86/freertos_help_managing_multiple_tasks_for_stepper/)
[8](https://worldscientificnews.com/wp-content/uploads/2025/01/WSN-199-2025-218-234.pdf)
[9](https://jte.edu.vn/index.php/jte/article/view/657)
[10](https://www.teachmemicro.com/multitask-with-esp32-and-freertos/)
[11](https://toptechboy.com/improving-accuracy-of-mpu6050-data-using-a-complimentary-filter/)
[12](https://journals.plos.org/plosone/article/file?id=10.1371%2Fjournal.pone.0280071&type=printable)
[13](https://randomnerdtutorials.com/esp32-freertos-arduino-tasks/)
[14](https://ijeee.edu.iq/Papers/Vol15-Issue2/172878.pdf)
[15](https://www.academia.edu/26878388/Artificial_Neural_Network_identification_and_control_of_the_inverted_pendulum)
[16](https://controllerstech.com/esp32-freertos-task-control-guide/)
[17](http://journalarticle.ukm.my/20600/1/24.pdf)
[18](https://www.arxiv.org/pdf/2502.00248.pdf)
[19](https://www.youtube.com/watch?v=V-RGB5yem-Q)
[20](https://www.youtube.com/watch?v=qmd6CVrlHOM)