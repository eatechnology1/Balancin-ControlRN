# 🤖 Balancín-ControlRN (ESP32-S3)

Firmware para un **robot balancín seguidor de línea** basado en **ESP32-S3**, que combina:

- Control clásico en cascada (PID) para el balanceo.
- Una **red neuronal feed-forward** que ajusta dinámicamente la ganancia proporcional del lazo de ángulo.
- Un **seguidor de línea** con arreglo de 8 sensores y PID independiente.

---

## ✨ Características principales

- 🧠 **Red neuronal** (Neural_Networks_FF + Dynamic_Array) que adapta `Kp` del lazo de ángulo en tiempo real.
- 🦾 **Control en cascada**:
  - Lazo de **ángulo** (PID) → genera referencia de velocidad.
  - Lazo de **velocidad** (PI) → genera PWM base de balanceo.
- 🎯 **Seguidor de línea**:
  - 8 sensores digitales.
  - PID que genera un `factor_giro` multiplicativo sobre el PWM base.
- 🕒 **Arquitectura RTOS (FreeRTOS)**:
  - `TaskBalanceo` (20 ms): lectura de MPU, encoders y control de balanceo.
  - `TaskLinea` (100 ms): lectura de sensores y PID de línea.
- 💾 **Calibración persistente** de la MPU con `Preferences` (NVS).
- 🧱 Código totalmente modular (headers separados por funcionalidad).

---

## 🧩 Estructura del código

Principales archivos del proyecto:

- `src/main.cpp`  
  Orquesta todo: inicializa hardware, carga calibración, llama a funciones de setup de cada módulo y crea las tareas RTOS.

- `include/config.h`  
  Definición de pines, constantes de PWM, encoders, sensores de línea y parámetros de control (Kp, Ki, Kd, límites, tiempos, etc.).

- `include/encoders.h`  
  - Variables globales `countR`, `countL`.  
  - ISRs `isr_RA` / `isr_LA` para los encoders.  
  - (Opcionalmente) funciones helper para cálculo de RPM.

- `include/motors.h`  
  - Configuración de PWM con LEDC (`setupMotorPWM`).  
  - Función `driveMotorsDifferential(pwmL, pwmR)` que aplica dirección y duty a cada motor.  

- `include/mpu_block.h`  
  - Objeto `MPU6050 mpu`.  
  - Funciones `saveCalibration`, `loadCalibration`, `calibrateMPU`.  
  - Filtro complementario para obtener `angleFiltered`.

- `include/nn_cascade_block.h`  
  - Objetos de la red neuronal (`Neural_Networks_FF`, `Dynamic_Array`).  
  - `initNeural()` para configurar estructura y funciones de activación.  
  - `cascada(angle, rpmLeft, rpmRight, dt)` que implementa el control en cascada + adaptación de `Kp_angle`.

- `include/line_follower_block.h`  
  - PID de línea (`pidLinea`).  
  - Lectura de los 8 sensores y cálculo de la posición (0..700).  
  - Tarea `TaskLinea` que actualiza `factor_giro`.

- `include/tasks_block.h`  
  - `TaskBalanceo` (control de balanceo + fusión con `factor_giro`).  
  - `stopRobotAndTasks` / `resumeRobotAndTasks`.  
  - Creación de tareas RTOS (`xTaskCreatePinnedToCore`).

Además, en la carpeta `test/` se guardan sketches y versiones anteriores (`*.old`) útiles como histórico y referencia durante la sintonía.

---

## 🔧 Hardware requerido

- 🧩 **Placa**: ESP32-S3 DevKitM-1 (o equivalente ESP32-S3).  
- 🎛️ **IMU**: MPU6050 (I2C, SDA=41, SCL=42 en la configuración actual).  
- ⚙️ **Driver de motores**: TB6612FNG (o similar):
  - Izquierdo: `AIN1`, `AIN2`, `PWMA`.
  - Derecho: `BIN1`, `BIN2`, `PWMB`.
  - `STBY` para habilitar/deshabilitar motores.
- 🚗 **Motores DC con encoders**: uno por rueda (canales A/B por lado).
- 📏 **Sensores de línea**: arreglo de 8 sensores digitales alineados bajo el robot.
- 🔘 **Botón de calibración**: `BTN_CAL` (activo en LOW) para iniciar la calibración de la MPU con el robot quieto y en vertical.

---

## 🧮 Flujo de funcionamiento

1. **Encendido / setup**
   - Inicializa Serial, I2C, MPU, PWM, encoders, red neuronal, PID de línea y tareas.
   - Intenta cargar la calibración de la MPU desde NVS y calcula un ángulo inicial.

2. **Tarea de balanceo (`TaskBalanceo`)**
   - Lee acelerómetro y giroscopio de la MPU6050.
   - Aplica filtro complementario → `angleFiltered`.
   - Lee y reinicia los contadores de encoders → RPM de cada rueda + filtro IIR.
   - Llama a `cascada(angleFiltered, rpmL_f, rpmR_f, dt)` para obtener el PWM base.
   - Combina PWM base con `factor_giro`:
     - `factorL = 1 + factor_giro`
     - `factorR = 1 - factor_giro`
   - Aplica `driveMotorsDifferential(pwmL, pwmR)`.

3. **Tarea de línea (`TaskLinea`)**
   - Lee los 8 sensores digitales (negro = LOW).  
   - Calcula la posición del centro de la línea (0..700).  
   - Ejecuta `pidLinea` y actualiza `factor_giro` en un rango acotado (ej. -0.40..0.40).

4. **Calibración de la MPU**
   - Al presionar el botón `BTN_CAL`:
     - Se llaman `stopRobotAndTasks()`.
     - `calibrateMPU()` toma múltiples muestras y calcula offsets.
     - Se guardan en NVS.
     - `resumeRobotAndTasks()` reanuda la operación.

---

## 🚀 Compilación y carga (PlatformIO)

1. Clonar el repositorio:

```
git clone https://github.com/eatechnology1/Balancin-ControlRN.git
cd Balancin-ControlRN
```

2. Compilar y cargar el firmware:

```
pio run -t upload
```

3. Abrir el monitor serie:

```
pio device monitor -b 115200
```

Asegúrate de tener disponibles las librerías **Neural_Networks_FF** y **Dynamic_Array** (en `lib/` o instaladas en PlatformIO), ya que el controlador neuronal depende de ellas.

---

## 🧭 Estado actual y próximos pasos

- ✅ Proyecto modularizado: control de balanceo, red neuronal, MPU, encoders, motores y seguidor de línea separados en headers.
- ✅ Integración con GitHub y control de versiones funcionando.
- 🔜 Posibles mejoras:
  - Mejor manejo cuando se pierde la línea (estrategias de búsqueda).
  - Ajuste fino de hiperparámetros de la red neuronal y límites de `Kp`.
  - Documentación extra con diagramas de bloques y fotos del robot.
