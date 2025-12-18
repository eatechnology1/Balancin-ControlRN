/*
  CascadaPrueba.ino - Documentation

  Overview
  --------
  This sketch implements a balancing two-wheeled robot with line-following
  capabilities on an ESP32-S3 using FreeRTOS tasks. The control architecture is
  a cascade (two-loop) controller: an inner angle controller (PID) that
  produces a speed reference, and an outer speed integrator that converts the
  speed error into motor PWM. A separate line-following PID computes a small
  multiplicative steering factor that modifies left/right PWM to follow a
  line. Two FreeRTOS tasks run concurrently:
    - TaskBalanceo (Core 1): reads IMU & encoders, computes balance controller,
      applies motor commands at ~50 Hz.
    - TaskLinea  (Core 0): reads 8 line sensors, runs PID, updates steering
      factor at ~10 Hz.

   Hardware mapping / pinout
  -------------------------
  Motors / H-bridge:
    AIN1  (pin 7)   - Left motor direction pin 1
    AIN2  (pin 6)   - Left motor direction pin 2
    PWMA  (pin 5)   - Left motor PWM (LEDC channel 0)
    BIN1  (pin 16)  - Right motor direction pin 1
    BIN2  (pin 17)  - Right motor direction pin 2
    PWMB  (pin 18)  - Right motor PWM (LEDC channel 1)
    STBY  (pin 15)  - Standby/enable for the H-bridge (active HIGH)

  Encoders (quadrature):
    R_A (pin 3)  - Right encoder channel A (interrupt)
    R_B (pin 46) - Right encoder channel B
    L_A (pin 9)  - Left encoder channel A (interrupt)
    L_B (pin 11) - Left encoder channel B

  Line sensors:
    NUM_SENSORS = 8
    sensorPins[] = {40, 38, 37, 36, 35, 0, 45, 48}
    (Black is assumed to be LOW; sensors read as digital inputs.)

  IMU:
    MPU6050 via I2C, Wire.begin(41, 42) (SDA=SCL pins specified).
    The MPU library provides getMotion6() and getAcceleration().

  PWM settings
  ------------
  - PWM frequency: PWM_FREQ = 20000 Hz
  - PWM resolution: 8 bits (0..255)
  - LEDC channels: CH_A = LEDC_CHANNEL_0, CH_B = LEDC_CHANNEL_1

  Important constants & scaling
  ----------------------------
  - CPR (encoder counts per revolution parameter constant defined as 44)
    NOTE: The actual pulses used for RPM in TaskBalanceo use PPR = 44.0 * 119.0
    (likely pulses per revolution after gearing); verify for your encoder/gearbox.
  - IMU scaling: accel / 16384, gyro / 131 (typical MPU6050 LSB scalings)
  - angle units: degrees (computed via atan2 and gyro integration)
  - RPM computation: pulses -> revolutions (using PPR) -> RPM with dt

  Concurrency & thread-safety
  ---------------------------
  - countR and countL are declared volatile and incremented/decremented in ISRs.
    The balance task reads and then clears these counts. Clearing without
    atomic protection introduces a potential race if an interrupt occurs while
    copying/resetting; on the ESP32 a 32-bit assignment is atomic, but if you
    change types or need absolute safety, disable interrupts or use
    portENTER_CRITICAL/portEXIT_CRITICAL around access.
  - factor_giro is volatile because TaskLinea updates it while TaskBalanceo
    reads it. The variable is a float; concurrent read/write without locking
    may be acceptable for occasional non-critical glitches, but consider an
    atomic or critical section if you observe instability.

  Interrupt service routines (ISRs)
  ---------------------------------
  - isr_RA() and isr_LA() handle encoder quadrature decoding by reading both
    A and B channels and incrementing/decrementing the corresponding count.
  - ISRs are marked IRAM_ATTR for performance on the ESP32.
  - attachInterrupt on channel A pins is set to CHANGE to capture both edges.

  Control architecture
  --------------------
  - cascada(angle, rpmLeft, rpmRight, dt):
      - angle PID: Kp_angle, Ki_angle, Kd_angle -> produces a speed reference
        (speed_ref). An anti-windup scheme is applied: integrator only
        accumulates if the PID output was not saturated in the direction of
        the error.
      - speed PID/integral: Kp_speed, Ki_speed -> produces base PWM to keep
        the robot moving to maintain the desired angle.
      - speed_integral is constrained for numeric stability.
      - If absolute angle > MAX_ANGLE, function returns 0 (safety stop).
  - Cascade sample time in TaskBalanceo: dt ~ 0.020 s (50 Hz).
  - The line-following PID's output (lineOutput) is constrained by
    pidLinea.SetOutputLimits(-0.6, 0.6) and interpreted as a multiplicative
    steering factor (factor_giro). Positive factor_giro causes the robot to
    steer left by increasing left motor PWM and decreasing right motor PWM.

  Motor actuation
  ---------------
  - driveMotorsDifferential(pwmL, pwmR):
      - STBY driven HIGH to enable bridge.
      - Direction bit logic: pwm>0 interpreted as "forward" with a specific
        AIN/BIN polarity assumption. Confirm wiring if directions are swapped.
      - PWM magnitude is abs(pwm) constrained to [0,255] and written to LEDC
        channels.
      - If your H-bridge expects inverted logic for direction or standby, adapt
        digitalWrite polarity.

  Line sensor processing
  ----------------------
  - The 8 sensors are read as digital. Assumes black is LOW (returns 1).
  - A weighted position is computed: pos += val * i * 100 producing a 0..700
    range for 8 sensors.
  - If all sensors read white (suma == 0) then factor_giro is set to 0
    (no steering). The code comments suggest adding logic to "search" for the
    line if required.

  IMU calibration
  ---------------
  - Averages 'samples' readings to compute ax_offset, ay_offset, az_offset, gx_offset.
  - az_offset is adjusted so gravity reads approximately +1g (16384).
  - Initial angleFiltered is computed from the initial averaged accel reading.

  Tuning parameters & suggested ranges
  ------------------------------------
  - Angle PID (primary stability):
      Kp_angle   = 80   (proportional; large impact on responsiveness)
      Ki_angle   = 0.9  (integral; helps maintain position)
      Kd_angle   = 1.8  (derivative; damps oscillations)
    These are highly hardware-dependent. Start smaller and increase Kp until
    oscillations then tune Kd and Ki.

  - Speed controller:
      Kp_speed = 0.7
      Ki_speed = 0.0004
    Integrator has limits: speed_integral constrained to [-150, 150].

  - Line PID:
      Kp_line = 40, Ki_line = 0, Kd_line = 12
      Output limits set to [-0.6, 0.6] (max ~60% multiplicative steering used
      here; effectively +/- 25% in comment was illustrative). Reduce Kp_line
      if response is too aggressive.

  Safety & sanity checks
  ----------------------
  - MAX_ANGLE = 40 degrees: if exceeded, cascada() returns 0 to stop motors.
    This is a soft safety; you should implement a hard safety cutoff (disable
    motors or STBY low) if the robot falls.
  - Verify encoder PPR and PPR multiplication factor (44.0 * 119.0) match your
    hardware; RPM calculations will be wrong otherwise.
  - Check motor direction wiring to ensure positive PWM corresponds to the
    expected forward direction.
  - Watch for integral windup; the code contains basic anti-windup but may need
    adjustments for your platform.

  Performance considerations
  --------------------------
  - Task stack sizes: TaskBalanceo uses 10000 bytes, TaskLinea uses 4000 bytes.
    Adjust if you see stack overflow. Priorities: balance (3) > line (1).
  - LEDC timers: both channels share a timer selection in setupMotorPWM. The
    code configures a timer for each call; on ESP32 be careful not to create
    conflicting timer configurations if reusing timer numbers.
  - PWM resolution 8-bit at 20 kHz may be fine for motors; reduce frequency if
    motor driver prefers lower switching frequency.

  Debugging
  ---------
  - Serial prints are included in both tasks for runtime diagnostics:
      - TaskBalanceo prints angleFiltered, PWM base, factor_giro, final PWMs.
      - TaskLinea prints line position, pid output, factor_giro periodically.
  - Use these to tune PID terms and observe response.

  Possible improvements
  ---------------------
  - Use atomic access or critical sections when reading/clearing encoder counts
    to avoid race conditions with ISRs.
  - Use a complementary or Kalman filter with proper timekeeping for angle
    estimation (current complementary filter uses fixed dt).
  - Add configurable parameters via serial or WiFi to tune PID values without
    recompiling.
  - Implement a line reacquisition behavior when sensors lose the line instead
    of simply setting factor_giro = 0.
  - Implement a safety watchdog: if angle goes outside a limit for more than a
    short duration, cut motors and signal an error.

  Build / run notes
  -----------------
  - Uses Arduino framework for ESP32-S3. Ensure MPU6050 library and PID_v1 are
    available and compatible.
  - Wire.begin(41, 42) sets custom I2C pins; verify these match board wiring.
  - The main loop() is empty because tasks are used for execution.

  Licensing / authorship
  ----------------------
  - This documentation describes the behavior of the provided sketch. Preserve
    original authorship and licensing when modifying or redistributing code.

  Contact / debugging checklist
  -----------------------------
  - If robot oscillates or falls:
      1) Reduce Kp_angle.
      2) Increase Kd_angle moderately.
      3) Verify IMU offsets and orientation.
      4) Ensure dt used in filter and PID equals real loop time.
  - If motors do not respond:
      1) Check STBY pin is HIGH and H-bridge wiring.
      2) Verify PWM channels configured and LEDC timers did not conflict.
      3) Confirm direction pin logic (HIGH/LOW) matches wiring.

*/
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <driver/ledc.h>
#include <PID_v1.h>

// ===================== PINES MOTORES ==========================
#define AIN1 7
#define AIN2 6
#define PWMA 5
#define BIN1 16
#define BIN2 17
#define PWMB 18
#define STBY 15

// ===================== PWM ====================================
#define PWM_FREQ 20000
#define PWM_RES 8
const ledc_channel_t CH_A = LEDC_CHANNEL_0;  // Motor Izquierdo (A)
const ledc_channel_t CH_B = LEDC_CHANNEL_1;  // Motor Derecho (B)

// ===================== PINES ENCODERS ==========================
#define R_A 3
#define R_B 46
#define L_A 9
#define L_B 11

volatile long countR = 0;
volatile long countL = 0;

const int CPR = 44;

// ===================== MPU ====================================
MPU6050 mpu;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0;

// ===================== FILTRO =================================
float angleFiltered = 0;

// ===================== CONTROL EN CASCADA (BALANCEO) ======================
float setpoint_angle = 0.0;
float Kp_angle = 80;
float Ki_angle = 0.9;
float Kd_angle = 1.8;

float angle_integral = 0;
float angle_prev_error = 0;

float Kp_speed = 0.7;
float Ki_speed = 0.0004;
float speed_integral = 0;

float MAX_ANGLE = 40;
float PWM_LIMIT = 255;

// ===================== SEGUIDOR DE LÍNEA ======================
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = { 40, 38, 37, 36, 35, 0, 45, 48 };
//const uint8_t sensorPins[NUM_SENSORS] = {48, 45, 0, 35, 36, 37, 38, 40};

// La salida del PID de línea.
// Valor de 0.0 es centro. +/- 0.25 es el giro máximo permitido.
volatile float factor_giro = 0.0;

// PID Línea
double lineInput = 0;
double lineOutput = 0;
double lineSetpoint = 350;  // centro ideal (0..700)

double Kp_line = 40;
double Ki_line = 0;
double Kd_line = 12;

// Nota: Puedes necesitar ajustar Kp_line si la respuesta es muy fuerte o suave.
PID pidLinea(&lineInput, &lineOutput, &lineSetpoint, Kp_line, Ki_line, Kd_line, DIRECT);

// ===================== ISRs ENCODER ============================
void IRAM_ATTR isr_RA() {
  if (digitalRead(R_A) == digitalRead(R_B)) countR++;
  else countR--;
}

void IRAM_ATTR isr_LA() {
  if (digitalRead(L_A) == digitalRead(L_B)) countL++;
  else countL--;
}

// ===================== PWM MOTOR ===============================
void setupMotorPWM(int pwmPin, int pwmChannel, int timerNum) {
  ledc_timer_config_t timer_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_8_BIT,
    .timer_num = (ledc_timer_t)timerNum,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };

  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = pwmPin,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = (ledc_channel_t)pwmChannel,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = (ledc_timer_t)timerNum,
    .duty = 0
  };

  ledc_channel_config(&channel_conf);
}

/**
 * @brief Aplica PWMs diferenciales a los dos motores.
 * @param pwmL PWM para el motor izquierdo (CH_A)
 * @param pwmR PWM para el motor derecho (CH_B)
 */
void driveMotorsDifferential(float pwmL, float pwmR) {
  digitalWrite(STBY, HIGH);

  // --- MOTOR IZQUIERDO (CH_A) ---
  int dutyL = constrain(abs(pwmL), 0, 255);

  // Lógica de dirección: Asumimos que HIGH/LOW para AIN2/AIN1 es adelante
  if (pwmL > 0) {  // Adelante
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  } else {  // Atrás
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, dutyL);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);

  //--- MOTOR DERECHO (CH_B) ---
  int dutyR = constrain(abs(pwmR), 0, 255);

  // Lógica de dirección: Asumimos que HIGH/LOW para BIN2/BIN1 es adelante
  if (pwmR > 0) {  // Adelante
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  } else {  // Atrás
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, dutyR);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);
}

// ===================== CONTROL EN CASCADA ======================
float cascada(float angle, float rpmLeft, float rpmRight, float dt) {
  float error = setpoint_angle - angle;
  float derivative = (error - angle_prev_error) / dt;

  float pid_unsat = Kp_angle * error + Ki_angle * angle_integral + Kd_angle * derivative;

  // Lazo interno: Salida es la REFERENCIA DE VELOCIDAD
  float speed_ref = constrain(pid_unsat, -250, 250);
  if (!((pid_unsat != speed_ref) && (error * pid_unsat) > 0)) {
    angle_integral += error * dt;
  }
  angle_prev_error = error;
  // Lazo externo: PID de Velocidad
  float speed_measured = (rpmLeft + rpmRight) * 0.5;
  float error_speed = speed_ref - speed_measured;

  // Salida es el PWM BASE que se necesita para mantener la velocidad
  float pwm_unsat = Kp_speed * error_speed + Ki_speed * speed_integral;
  float pwm_balanceo_base = constrain(pwm_unsat, -PWM_LIMIT, PWM_LIMIT);

  if (!((pwm_unsat != pwm_balanceo_base) && (error_speed * pwm_unsat) > 0)) {
    speed_integral += error_speed * dt;
  }

  speed_integral = constrain(speed_integral, -150, 150);

  if (abs(angle) > MAX_ANGLE) return 0;
  return pwm_balanceo_base;
}

// ===================== TASK BALANCEO ===========================
void TaskBalanceo(void *pvParameters) {
  const TickType_t dt_task = pdMS_TO_TICKS(20);

  while (true) {
    // La MPU, el filtro y el cálculo de RPM quedan IGUAL
    // ... (Código MPU/Filtro/RPM) ...

    float dt = 0.020;
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    float accelY = (ay - ay_offset) / 16384.0;
    float accelZ = (az - az_offset) / 16384.0;
    float gyroX = (gx - gx_offset) / 131.0;
    float angleAcc = atan2(accelY, accelZ) * 180.0 / PI;
    angleFiltered = 0.98 * (angleFiltered + gyroX * dt) + 0.02 * angleAcc;

    long pulsesR = countR;
    long pulsesL = countL;
    countR = 0;
    countL = 0;

    const float PPR = 44.0 * 119.0;
    float revR = pulsesR / PPR;
    float revL = pulsesL / PPR;
    float rpmR = (revR / dt) * 60.0;
    float rpmL = (revL / dt) * 60.0;

    static float rpmR_f = 0, rpmL_f = 0;
    rpmR_f = 0.7 * rpmR_f + 0.3 * rpmR;
    rpmL_f = 0.7 * rpmL_f + 0.3 * rpmL;

    // ---- CONTROL DE BALANCEO: Obtiene el PWM BASE ----
    float pwm_balanceo_base = cascada(angleFiltered, rpmL_f, rpmR_f, dt);

    // ---- FUSIÓN MULTIPLICATIVA DE LA LÍNEA ----
    // La magnitud del factor_giro (ej. 0.25) determina el giro máximo.
    // Si factor_giro > 0, necesita girar a la izquierda (Aumentar L, reducir R).
    // Si factor_giro < 0, necesita girar a la derecha (Reducir L, aumentar R).

    float factorL = 1.0 + factor_giro;
    float factorR = 1.0 - factor_giro;

    float pwm_finalL = pwm_balanceo_base * factorL;
    float pwm_finalR = pwm_balanceo_base * factorR;

    // Recortar al límite absoluto de PWM_LIMIT (255)
    pwm_finalL = constrain(pwm_finalL, -PWM_LIMIT, PWM_LIMIT);
    pwm_finalR = constrain(pwm_finalR, -PWM_LIMIT, PWM_LIMIT);

    // ---- ACTUACIÓN ----
    driveMotorsDifferential(pwm_finalL, pwm_finalR);

    // Debug
    Serial.print("Ang: ");
    Serial.print(angleFiltered);
    Serial.print(" | PWM Base: ");
    Serial.print(pwm_balanceo_base);
    Serial.print(" | Factor G: ");
    Serial.print(factor_giro);
    Serial.print(" | PWM L: ");
    Serial.print(pwm_finalL);
    Serial.print(" | PWM R: ");
    Serial.println(pwm_finalR);

    vTaskDelay(dt_task);
  }
}

// ===================== TASK LÍNEA ==============================
void TaskLinea(void *pvParameters) {
  pidLinea.SetMode(AUTOMATIC);
  // La salida se limita a un valor que represente el porcentaje de giro máximo.
  pidLinea.SetOutputLimits(-0.6, 0.6);  // Ej. Máx. 25% de corrección.
  pidLinea.SetSampleTime(100);

  while (true) {
    int suma = 0;
    int pos = 0;
    int num_blancos = 0;

    for (int i = 0; i < NUM_SENSORS; i++) {
      // Asumes negro=LOW (1)
      int val = digitalRead(sensorPins[i]) == LOW ? 1 : 0;
      suma += val;
      pos += val * i * 100;  // Posición ponderada (0, 100, 200... 700)
      if (val == 0) num_blancos++;
    }

    if (suma > 0) {
      lineInput = (double)(pos / suma);  // 0 - 700
      pidLinea.Compute();
      factor_giro = (float)lineOutput;  // La corrección PID es el factor

    } else {
      // Si no detecta la línea (suma=0), el robot avanza recto (factor_giro = 0)
      factor_giro = 0;
    }

    // NOTA: Si necesitas que el robot "busque" la línea cuando se pierde,
    // tendrías que usar un PID de "Pérdida de Línea" aquí, manteniendo la
    // última dirección conocida. Por ahora, se detiene la corrección de giro.

    // Debug línea
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.print("LINE POS: ");
      Serial.print(lineInput);
      Serial.print(" | PID OUT: ");
      Serial.print(lineOutput);
      Serial.print(" | FACTOR GIRO: ");
      Serial.println(factor_giro);
    }

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// ===================== SETUP ===================================
void setup() {
  Serial.begin(115200);
  Wire.begin(41, 42);
  mpu.initialize();

  // Motores
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  setupMotorPWM(PWMA, CH_A, 0);
  setupMotorPWM(PWMB, CH_B, 1);

  // Encoders
  pinMode(R_A, INPUT_PULLUP);
  pinMode(R_B, INPUT_PULLUP);
  pinMode(L_A, INPUT_PULLUP);
  pinMode(L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(R_A), isr_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_A), isr_LA, CHANGE);

  // Sensores de línea
  for (int i = 0; i < NUM_SENSORS; i++) pinMode(sensorPins[i], INPUT);

  // Calibración MPU (sin cambios)
  // ... (código de calibración) ...
  long ax_sum = 0,
       ay_sum = 0, az_sum = 0, gx_sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++) {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    ax_sum += ax;
    ay_sum += ay;
    az_sum += az;
    gx_sum += gx;
    delay(3);
  }

  ax_offset = ax_sum / samples;
  ay_offset = ay_sum / samples;
  az_offset = (az_sum / samples) - 16384;
  gx_offset = gx_sum / samples;

  int16_t ax_init, ay_init, az_init;
  mpu.getAcceleration(&ax_init, &ay_init, &az_init);
  angleFiltered = atan2((ay_init - ay_offset) / 16384.0, (az_init - az_offset) / 16384.0) * 180.0 / PI;
  // Crear tarea Balanceo en Core 1
  xTaskCreatePinnedToCore(
    TaskBalanceo,
    "TaskBalanceo",
    10000,
    NULL,
    3,
    NULL,
    1);

  // Crear tarea Línea en Core 0
    xTaskCreatePinnedToCore(
      TaskLinea,
      "TaskLinea",
      4000,
      NULL,
      1,
      NULL,
      0);

  Serial.println("Balanceo corriendo en Core 1. Seguidor linea en Core 0.");
}

void loop() {
  // El loop principal permanece vacío ya que FreeRTOS maneja todo.
}