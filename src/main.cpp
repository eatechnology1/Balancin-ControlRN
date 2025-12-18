/**
 * @file main.cpp
 * @brief Balancing line-following robot firmware for ESP32-S3 using MPU6050, encoders,
 *        PID controllers and an adaptive feed-forward neural network.
 *
 * Overview
 * --------
 * This firmware implements a two-task RTOS-based control system:
 *  - TaskBalanceo: high-rate balancing controller (≈20 ms loop) that reads MPU6050,
 *                  filters angle, computes wheel RPM from incremental encoders,
 *                  computes a cascade controller output (angle -> speed -> PWM)
 *                  and applies differential motor commands.
 *  - TaskLinea:    lower-rate line-following task (100 ms) that reads an 8-sensor
 *                  digital array to compute a position estimate and runs a PID to
 *                  provide a multiplicative steering factor (factor_giro).
 *
 * The balancing controller uses a small feed-forward neural network (Neural_Networks_FF)
 * that is trained online to adapt the proportional gain of the angle controller.
 * PID is used at two levels: an inner angle-to-speed PID (tuned by NN output) and
 * an outer speed-to-PWM PI. A separate PID is used for line following to produce
 * a steering factor that multiplies the base balancing PWM differentially between
 * the left/right motors.
 *
 * Hardware / Pinout Summary
 * -------------------------
 * - BTN_CAL (GPIO 10)      : Push-button to start MPU calibration (active LOW).
 * - Motor driver control:
 *     AIN1 (7), AIN2 (6), PWMA (5)  : Left motor direction pins and PWM pin
 *     BIN1 (16), BIN2 (17), PWMB (18): Right motor direction pins and PWM pin
 *     STBY (15)                      : Standby / enable pin for motor driver
 * - PWM channels:
 *     CH_A = LEDC_CHANNEL_0 (PWMA)
 *     CH_B = LEDC_CHANNEL_1 (PWMB)
 *     PWM_FREQ = 20 kHz, PWM_RES = 8 bits
 * - Encoders (quadrature-like using two GPIOs each):
 *     R_A (3), R_B (46) : Right encoder
 *     L_A (9), L_B (11) : Left encoder
 * - Line sensors:
 *     sensorPins[8] = {48, 45, 0, 35, 36, 37, 38, 40}
 *
 * Constants and Units
 * -------------------
 * - MPU accelerometer scale: raw / 16384.0 -> g units
 * - MPU gyro scale: raw / 131.0 -> deg/s
 * - Complementary filter: angleFiltered = 0.98*(prev + gyro*dt) + 0.02*angleAcc
 * - Encoder PPR/PPR-derived:
 *     CPR = 44 (counts per encoder revolution raw)
 *     PPR used in code = 44.0 * 119.0 (likely gearing / pulses per wheel rev)
 * - Timing:
 *     TaskBalanceo loop dt ≈ 0.020 s (20 ms)
 *     TaskLinea loop dt   = 100 ms (PID sample time set to 100 ms)
 *
 * Globals of interest (summary)
 * -----------------------------
 * - Neural network:
 *     net : Neural_Networks_FF instance
 *     XI  : Dynamic_Array inputs for NN (1 x 3)
 *     D   : Dynamic_Array desired outputs (1 x 1)
 *     EST : Dynamic_Array network structure description
 *     FAC_STR : Dynamic_Array of activation function names
 *     posicion : Dynamic_Array used as an extra NN input / aux
 *     net.LearningRate initially set to 2.25 for online learning
 *     setPoint = 0.700 (used as desired NN output / reference)
 * - Balancing control:
 *     setpoint_angle = 0.7 (deg? unit scaled in code)
 *     Kp_angle, Ki_angle, Kd_angle : base gains; Kp_angle is updated by NN output
 *     Kp_speed, Ki_speed : speed PI gains
 *     angle_integral, speed_integral : integrators with limits
 *     MAX_ANGLE : 40 deg - safety cutoff (returns 0 PWM if exceeded)
 *     PWM_LIMIT : 255 - output clipping for PWM duty
 * - Line follower:
 *     NUM_SENSORS = 8
 *     factor_giro (volatile float) : [-0.40..0.40] steering factor produced by PID
 *     lineSetpoint default = 350 (center of 0..700 sensor position scale)
 *     PID pidLinea configured with Kp_line, Ki_line, Kd_line
 * - Encoders:
 *     volatile long countR, countL : incremented in ISRs and read/reset in TaskBalanceo
 *
 * Key Functions and Behavior
 * --------------------------
 * - isr_RA(), isr_LA()
 *     - Attached to CHANGE interrupts on encoder A channels.
 *     - Determine direction by comparing A and B channel states and increment/decrement counts.
 *     - Marked IRAM_ATTR (safe for ISRs on ESP32). Avoid blocking calls, Serial, or allocations.
 *
 * - setupMotorPWM(pwmPin, pwmChannel, timerNum)
 *     - Configures LEDC timer & channel for the given GPIO and channel.
 *     - Uses LEDC_LOW_SPEED_MODE and 8-bit resolution at PWM_FREQ.
 *
 * - driveMotorsDifferential(pwmL, pwmR)
 *     - Applies direction logic (HIGH/LOW AINx/BINx) and sets LEDC duties for left/right.
 *     - Accepts signed floats; sign indicates direction, magnitude clipped to 0..255.
 *     - Asserts STBY HIGH before enabling motors.
 *
 * - cascada(angle, rpmLeft, rpmRight, dt)
 *     - Implements cascade controller:
 *         1) Compute angle error and prepare NN input XI = [error, delta_error, delta_error_prev]
 *         2) Normalize inputs and perform online training of net using desired D (setPoint/100)
 *         3) Read NN output and scale to adjust Kp_angle dynamically
 *         4) Compute angle PID (P/ I/ D) to obtain speed reference (speed_ref)
 *         5) Inner speed PI computes pwm_unsat -> pwm_balanceo_base (clipped)
 *         6) Integrators updated only if not saturating with sign consistency
 *     - Returns base PWM for balance (signed), clipped to PWM_LIMIT; returns 0 if |angle| > MAX_ANGLE.
 *     - Side effects: updates global integrators, error history and neural net internal state.
 *
 * - saveCalibration(), loadCalibration()
 *     - Use Preferences to persist and restore ax_off, ay_off, az_off, gx_off under namespace "mpu".
 *     - loadCalibration returns false if keys don't exist.
 *
 * - calibrateMPU()
 *     - Collects samples (default samples=500) using mpu.getMotion6, computes average offsets:
 *         ax_offset, ay_offset, az_offset (with gravity compensation), gx_offset
 *     - Calls saveCalibration() and prints status to Serial.
 *     - Uses blocking delays and is intended to be called while tasks are suspended.
 *
 * - TaskBalanceo(void *pvParameters)
 *     - RTOS task pinned to a core (created with xTaskCreatePinnedToCore).
 *     - Loop period dt_task = 20 ms (pdMS_TO_TICKS).
 *     - Reads MPU with mpu.getMotion6, computes complementary filter angleFiltered.
 *     - Reads and zeroes encoder counts (countR, countL) to compute RPM per dt.
 *     - Applies low-pass smoothing to RPM readings (simple IIR).
 *     - Calls cascada(...) to get base PWM, applies multiplicative steering factor:
 *         factorL = 1 + factor_giro; factorR = 1 - factor_giro
 *       to obtain pwm_finalL/R, clips them and calls driveMotorsDifferential().
 *     - Prints debug line with angle, pwm base, factor, final PWM left/right.
 *     - Important: countR/countL are reset in task context after atomic read; ensure ISR uses volatile.
 *
 * - TaskLinea(void *pvParameters)
 *     - Sets PID pidLinea in AUTOMATIC mode, output limits [-0.40, 0.40], sample time 100 ms.
 *     - Polls 8 digital sensors: treats LOW as black (value=1), builds weighted position in [0..700].
 *     - If any sensor sees the line (suma>0) computes lineInput = pos/suma and runs pidLinea.Compute(),
 *       then sets volatile factor_giro = lineOutput. If no sensors detect line, factor_giro = 0.
 *     - Optionally prints debug lines every 200 ms.
 *
 * - stopRobotAndTasks()
 *     - Immediately sets LEDC duties to 0 for both channels and sets STBY LOW.
 *     - Suspends TaskBalanceo and TaskLinea (if handles not NULL).
 *     - Small delay to allow suspension to take effect.
 *
 * - resumeRobotAndTasks()
 *     - Resumes TaskLinea and TaskBalanceo then sets STBY HIGH.
 *     - Small delay to allow tasks to stabilize before motion.
 *
 * - setup()
 *     - Initializes Serial, Wire (I2C) with specified SDA/SCL pins, MPU, and pinMode for all pins.
 *     - Loads stored calibration if present and computes an initial angleFiltered if calibration loaded.
 *     - Configures motor PWM, encoder interrupts, neural network arrays & structure, and creates tasks:
 *         xTaskCreatePinnedToCore(TaskBalanceo, "TaskBalanceo", 10000, NULL, 3, &taskBalHandle, 1);
 *         xTaskCreatePinnedToCore(TaskLinea,   "TaskLinea",   4000, NULL, 1, &taskLineHandle, 0);
 *       Note: stack size and priorities were chosen empirically; adjust if running out of stack.
 *
 * - loop()
 *     - Polls BTN_CAL (active LOW). When pressed:
 *         1) Calls stopRobotAndTasks()
 *         2) Calls calibrateMPU() (blocking)
 *         3) Calls resumeRobotAndTasks()
 *       Includes simple debouncing delays to avoid re-triggering.
 *
 * Design Notes, Assumptions & Safety
 * ---------------------------------
 * - ISRs: keep short. No malloc/new/Serial/long operations inside isr_RA/isr_LA.
 * - countR / countL are volatile and used as delta counters; they are reset in TaskBalanceo.
 * - When calibrating, motors and tasks must be stopped to avoid motion and measurement noise.
 * - NN online learning uses a custom Neural_Networks_FF library — ensure its memory usage is
 *   compatible with the chosen task stack sizes and the ESP32 heap.
 * - The choice of complementary filter constants and PID/NN gains are critical. Provide safe defaults
 *   and test with the robot fixed (wheels blocked or held) before running free.
 * - MAX_ANGLE prevents large-tilt actuation; tune to robot geometry. Consider emergency STOP condition
 *   to shut down motor power (STBY LOW) if angle becomes dangerous.
 * - driveMotorsDifferential assumes a mapping of direction pins; verify hardware wiring before powering up.
 * - LEDC duty is set directly using raw 8-bit values (0..255). If motor driver expects other scaling,
 *   convert accordingly.
 *
 * Tuning Recommendations
 * ----------------------
 * - Start with motors disabled (STBY LOW) and verify sensor readings (Serial print angleFiltered).
 * - Calibrate MPU with the robot stationary and upright; verify ax_offset/ay_offset/az_offset produce
 *   a reasonable initial angle.
 * - Tune line PID with the robot on a stand: try reduced Kp_line to avoid oscillatory behavior.
 * - For balancing, test with small Kp_angle/gains and increase gradually. Because Kp_angle is adapted
 *   by the NN, monitor its output to ensure it remains within safe bounds.
 * - If integrators wind up, check anti-windup logic: integrators are only updated if unsaturated
 *   or sign-check prevents wrong accumulation.
 *
 * Persistence
 * -----------
 * - Calibration values saved in NVS/Preferences under namespace "mpu" with keys:
 *     "ax_off", "ay_off", "az_off", "gx_off"
 * - Use loadCalibration/saveCalibration to restore/commit calibration.
 *
 * Possible Improvements
 * ---------------------
 * - Add a watchdog and fail-safe to kill motors on task crash or stack overflow.
 * - Improve line-loss behavior: remember last factor_giro and attempt recovery sweeps.
 * - Use hardware timer for precise dt measurement instead of assuming fixed dt.
 * - Move heavy NN training out of hard real-time path or limit training frequency.
 * - Protect shared variables accessed in ISRs and tasks using critical sections if multi-word values needed.
 *
 * Concurrency & RTOS Notes
 * ------------------------
 * - Tasks are created pinned to cores; verify core assignment fits other application constraints.
 * - Suspending/resuming tasks is used during calibration; consider using notifications or semaphores
 *   if finer coordination is needed.
 * - Ensure the TaskBalanceo stack (10000 bytes configured) is sufficient for NN calls and any temporary
 *   allocations. Reduce blocking calls and dynamic memory in that context.
 *
 * Debugging Tips
 * --------------
 * - Use Serial prints in TaskBalanceo and TaskLinea to monitor angle, PWM, factor_giro, and PID outputs.
 * - If encoders give weird RPM values, verify PPR and that the encoder interrupt logic for direction is correct.
 * - Check I2C wiring and mpu.initialize() return values; validate raw accelerometer and gyro values prior to filtering.
 *
 * Licensing / Attribution
 * -----------------------
 * - This comment documents the provided source and design; adjust attributions/comments as required by project policy.
 *
 */
#include <Arduino.h>
#include "Neural_Networks_FF.h"
#include <Wire.h>
#include <MPU6050.h>
#include <driver/ledc.h>
#include <PID_v1.h>
#include <Preferences.h>

// ================= BOTÓN ===============
#define BTN_CAL 10
Preferences prefs;
bool calibratedOnce = false;

// Handles para poder suspender/reanudar tareas
TaskHandle_t taskBalHandle = NULL;
TaskHandle_t taskLineHandle = NULL;

// ===================== PINES MOTORES ==========================
#define AIN1 7
#define AIN2 6
#define PWMA 5
#define BIN1 16
#define BIN2 17
#define PWMB 18
#define STBY 15

// Contructor de la red neuronal
Neural_Networks_FF net = Neural_Networks_FF();
// Array dinamico para ingresar las entradas a la red
Dynamic_Array XI = Dynamic_Array();
// Array dinamico para ingresar las salidas deseadas
Dynamic_Array D = Dynamic_Array();
// Array dinamico tipo string para ingresar las funciones de activacion por capa
Dynamic_Array FAC_STR = Dynamic_Array();
// Array dinamico para ingresar la estructura de la red como vector
Dynamic_Array EST = Dynamic_Array();
Dynamic_Array posicion = Dynamic_Array();

// ===================== PWM ====================================
#define PWM_FREQ 20000
#define PWM_RES 8
const ledc_channel_t CH_A = LEDC_CHANNEL_0; // Motor Izquierdo (A)
const ledc_channel_t CH_B = LEDC_CHANNEL_1; // Motor Derecho (B)

// ===================== PINES ENCODERS ==========================
#define R_A 3
#define R_B 46
#define L_A 9
#define L_B 11

volatile long countR = 0;
volatile long countL = 0;

const int CPR = 44;
// =====================  Variables para el control RN ==========================

float vel = 0.00;
float error = 0.00;
float error_1 = 0.00;
float error_2 = 0.00;
float setPoint = 0.000;
float Output = 0.00;

// ===================== MPU ====================================
MPU6050 mpu;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0;

// ===================== FILTRO =================================
float angleFiltered = 0;

// ===================== CONTROL EN CASCADA (BALANCEO) ======================
float setpoint_angle = 0.0;
float Kp_angle = 85;  // 80
float Ki_angle = 1.0; // 1
float Kd_angle = 1.6; // Pruebalo 1

float angle_integral = 0;
float angle_prev_error = 0;

float Kp_speed = 0.8; // 0.8
float Ki_speed = 0.000001;
float speed_integral = 0;

float MAX_ANGLE = 40;
float PWM_LIMIT = 255;

// ===================== SEGUIDOR DE LÍNEA ======================
#define NUM_SENSORS 8
// const uint8_t sensorPins[NUM_SENSORS] = { 40, 38, 37, 36, 35, 0, 45, 48 };
const uint8_t sensorPins[NUM_SENSORS] = {48, 45, 0, 35, 36, 37, 38, 40};

// La salida del PID de línea.
// Valor de 0.0 es centro. +/- 0.25 es el giro máximo permitido.
volatile float factor_giro = 0.0;

// PID Línea
double lineInput = 0;
double lineOutput = 0;
double lineSetpoint = 350; // centro ideal (0..700)

double Kp_line = 30;
double Ki_line = 0.001;
double Kd_line = 10;

// Nota: Puedes necesitar ajustar Kp_line si la respuesta es muy fuerte o suave.
PID pidLinea(&lineInput, &lineOutput, &lineSetpoint, Kp_line, Ki_line, Kd_line, DIRECT);

// ===================== ISRs ENCODER ============================
void IRAM_ATTR isr_RA()
{
  if (digitalRead(R_A) == digitalRead(R_B))
    countR++;
  else
    countR--;
}

void IRAM_ATTR isr_LA()
{
  if (digitalRead(L_A) == digitalRead(L_B))
    countL++;
  else
    countL--;
}

// ===================== PWM MOTOR ===============================
void setupMotorPWM(int pwmPin, int pwmChannel, int timerNum)
{
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_8_BIT,
      .timer_num = (ledc_timer_t)timerNum,
      .freq_hz = PWM_FREQ,
      .clk_cfg = LEDC_AUTO_CLK};

  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
      .gpio_num = pwmPin,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = (ledc_channel_t)pwmChannel,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = (ledc_timer_t)timerNum,
      .duty = 0};

  ledc_channel_config(&channel_conf);
}

/**
 * @brief Aplica PWMs diferenciales a los dos motores.
 * @param pwmL PWM para el motor izquierdo (CH_A)
 * @param pwmR PWM para el motor derecho (CH_B)
 */
void driveMotorsDifferential(float pwmL, float pwmR)
{
  digitalWrite(STBY, HIGH);

  // --- MOTOR IZQUIERDO (CH_A) ---
  int dutyL = constrain(abs(pwmL), 0, 255);

  // Lógica de dirección: Asumimos que HIGH/LOW para AIN2/AIN1 es adelante
  if (pwmL > 0)
  { // Adelante
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  else
  { // Atrás
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  }
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, dutyL);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);

  //--- MOTOR DERECHO (CH_B) ---
  int dutyR = constrain(abs(pwmR), 0, 255);

  // Lógica de dirección: Asumimos que HIGH/LOW para BIN2/BIN1 es adelante
  if (pwmR > 0)
  { // Adelante
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }
  else
  { // Atrás
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  }
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, dutyR);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);
}

// ===================== CONTROL EN CASCADA ======================
float cascada(float angle, float rpmLeft, float rpmRight, float dt)
{
  float derivative = (error - angle_prev_error) / dt;
  posicion.WriteArray_2D(0, 0, angle / 100.00);
  error = setpoint_angle - angle;
  XI.WriteArray_2D(0, 0, error);
  XI.WriteArray_2D(0, 1, error - error_1);
  XI.WriteArray_2D(0, 2, error_1 - error_2);
  // net.DATA_CENTERING(XI);
  net.NORMALIZE_DATA(XI, 100.00);
  D.WriteArray_2D(0, 0, setPoint / 100.00);
  net.TRAIN_NET_ONLINE(XI, D, posicion);
  Output = net.y.ReadArray_2D(net.y.GetRows() - 1, 0);
  error_2 = error_1;
  error_1 = error;
  Kp_angle = Output * 100;

  float pid_unsat = Kp_angle * error + Ki_angle * angle_integral + Kd_angle * derivative;

  // Lazo interno: Salida es la REFERENCIA DE VELOCIDAD
  float speed_ref = constrain(pid_unsat, -300, 300);
  if (!((pid_unsat != speed_ref) && (error * pid_unsat) > 0))
  {
    angle_integral += error * dt;
  }
  angle_prev_error = error;
  // Lazo externo: PID de Velocidad
  float speed_measured = (rpmLeft + rpmRight) * 0.5;
  float error_speed = speed_ref - speed_measured;

  // Salida es el PWM BASE que se necesita para mantener la velocidad
  float pwm_unsat = Kp_speed * error_speed + Ki_speed * speed_integral;
  float pwm_balanceo_base = constrain(pwm_unsat, -PWM_LIMIT, PWM_LIMIT);

  if (!((pwm_unsat != pwm_balanceo_base) && (error_speed * pwm_unsat) > 0))
  {
    speed_integral += error_speed * dt;
  }

  speed_integral = constrain(speed_integral, -150, 150);

  if (abs(angle) > MAX_ANGLE)
    return 0;
  return pwm_balanceo_base;
}

void saveCalibration()
{
  prefs.begin("mpu", false);
  prefs.putFloat("ax_off", ax_offset);
  prefs.putFloat("ay_off", ay_offset);
  prefs.putFloat("az_off", az_offset);
  prefs.putFloat("gx_off", gx_offset);
  prefs.end();
  Serial.println(">> Calibración guardada.");
}

bool loadCalibration()
{
  prefs.begin("mpu", true);
  if (!prefs.isKey("ax_off"))
  {
    prefs.end();
    return false;
  }
  ax_offset = prefs.getFloat("ax_off");
  ay_offset = prefs.getFloat("ay_off");
  az_offset = prefs.getFloat("az_off");
  gx_offset = prefs.getFloat("gx_off");
  prefs.end();
  return true;
}

void calibrateMPU()
{
  Serial.println(">> Iniciando calibración MPU...");

  long ax_sum = 0, ay_sum = 0, az_sum = 0, gx_sum = 0;
  const int samples = 500;

  for (int i = 0; i < samples; i++)
  {
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

  saveCalibration();
  Serial.println(">> Calibración MPU COMPLETA.");
}

// ===================== TASK BALANCEO ===========================
void TaskBalanceo(void *pvParameters)
{
  const TickType_t dt_task = pdMS_TO_TICKS(20);

  while (true)
  {
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
void TaskLinea(void *pvParameters)
{
  pidLinea.SetMode(AUTOMATIC);
  // La salida se limita a un valor que represente el porcentaje de giro máximo.
  pidLinea.SetOutputLimits(-0.40, 0.40); // Ej. Máx. 25% de corrección.
  pidLinea.SetSampleTime(100);

  while (true)
  {
    int suma = 0;
    int pos = 0;
    int num_blancos = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      // Asumes negro=LOW (1)
      int val = digitalRead(sensorPins[i]) == LOW ? 1 : 0;
      suma += val;
      pos += val * i * 100; // Posición ponderada (0, 100, 200... 700)
      if (val == 0)
        num_blancos++;
    }

    if (suma > 0)
    {
      lineInput = (double)(pos / suma); // 0 - 700
      pidLinea.Compute();
      factor_giro = (float)lineOutput; // La corrección PID es el factor
    }
    else
    {
      // Si no detecta la línea (suma=0), el robot avanza recto (factor_giro = 0)
      factor_giro = 0;
    }

    // NOTA: Si necesitas que el robot "busque" la línea cuando se pierde,
    // tendrías que usar un PID de "Pérdida de Línea" aquí, manteniendo la
    // última dirección conocida. Por ahora, se detiene la corrección de giro.

    // Debug línea
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200)
    {
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

// Detiene físicamente los motores y suspende las tareas relevantes
void stopRobotAndTasks()
{
  // Apagar PWM (poner duty 0)
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);

  // Opcional: desactivar STBY si tu driver lo soporta (evita que se energicen los H-bridges)
  digitalWrite(STBY, LOW);

  // Suspender sólo las tareas que usamos
  if (taskBalHandle != NULL)
    vTaskSuspend(taskBalHandle);
  if (taskLineHandle != NULL)
    vTaskSuspend(taskLineHandle);

  // Pequeña espera para asegurar que tareas se suspendan
  delay(20);
}

// Reactiva tareas y vuelve a permitir movimiento
void resumeRobotAndTasks()
{
  // Reactivar tareas primero (para que arranquen sus loops)
  if (taskLineHandle != NULL)
    vTaskResume(taskLineHandle);
  if (taskBalHandle != NULL)
    vTaskResume(taskBalHandle);

  // Rehabilitar STBY para permitir motores
  digitalWrite(STBY, HIGH);

  // Espera para que las tareas estabilicen antes de permitir movimiento
  delay(20);
}

// ===================== SETUP ===================================
void setup()
{
  Serial.begin(115200);
  Wire.begin(41, 42);
  mpu.initialize();

  pinMode(BTN_CAL, INPUT_PULLUP); // Botón hacia GND

  // ======== CARGAR CALIBRACIÓN GUARDADA =========
  if (loadCalibration())
  {
    Serial.println(">> Calibración cargada de memoria.");
  }
  else
  {
    Serial.println(">> No existe calibración guardada. Necesitas presionar el botón.");
  }

  // ======== Inicializar Motores y Encoders ========
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  setupMotorPWM(PWMA, CH_A, 0);
  setupMotorPWM(PWMB, CH_B, 1);

  pinMode(R_A, INPUT_PULLUP);
  pinMode(R_B, INPUT_PULLUP);
  pinMode(L_A, INPUT_PULLUP);
  pinMode(L_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(R_A), isr_RA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L_A), isr_LA, CHANGE);
  // CONFIGURACION DE LA RED NEURONAL
  // Crear e inicializar el array de los datos de entrada
  XI.NewArray_2D(1, 3);
  XI.ToinitializeArray_2D(0.0);
  // Crear e inicializar el array de salidas deseadas
  D.NewArray_2D(1, 1);
  D.ToinitializeArray_2D(0.0);
  // Columnas de XI
  byte XC = XI.GetColumns();
  // Filas de D
  byte DF = D.GetRows();
  // Filas de XI
  byte XF = XI.GetRows();
  // Definomos numero de capas
  byte L = 3;
  // Creamos e inicializamos el array donde se guardara
  // la estructura de la red
  EST.NewArray_2D(1, L);
  EST.ToinitializeArray_2D(0.0);
  float NC = EST.GetColumns();
  // Vector que contiene la estructura de la red
  float V[] = {XC, 3, DF};
  // Copiamos el vector al array
  for (byte i = 0; i < NC; i++)
  {
    EST.WriteArray_2D(0, i, V[i]);
  }
  // creamos array tipo string para pasar
  // las funciones de activacion
  FAC_STR.CharNewArray_1D(L);
  // Vector que contine las funciones de activacion
  char *myStrings[] = {"logsig", "logsig", "poslin_lim"};
  // Copiamos el vector al array
  for (byte i = 0; i < FAC_STR.GetRows(); i++)
  {
    FAC_STR.CharWriteArray_1D(i, myStrings[i]);
  }
  net.dposlin_limits(1, -1);
  // Creamos la red
  net.LearningRate = 0.00;
  net.FEED_FORWARD_NET(EST, FAC_STR);
  posicion.NewArray_2D(1, 1);
  posicion.ToinitializeArray_2D(0.0);
  // Variable de entrenamiento
  net.LearningRate = 2.25; // 0.025;
  setPoint = 0.700;
  // Sensores de línea
  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensorPins[i], INPUT);

  // Si ya existe calibración → calcula ángulo inicial
  if (loadCalibration())
  {
    int16_t ax_init, ay_init, az_init;
    mpu.getAcceleration(&ax_init, &ay_init, &az_init);
    angleFiltered = atan2(
                        (ay_init - ay_offset) / 16384.0,
                        (az_init - az_offset) / 16384.0) *
                    180.0 / PI;
  }

  // ======== CREAR TAREAS ========
  xTaskCreatePinnedToCore(TaskBalanceo, "TaskBalanceo", 10000, NULL, 3, &taskBalHandle, 1);
  xTaskCreatePinnedToCore(TaskLinea, "TaskLinea", 4000, NULL, 1, &taskLineHandle, 0);

  Serial.println("Robot listo. Presiona el botón para calibrar la MPU.");
}

void loop()
{
  // Detecta si el botón está presionado (pin 10 a GND)
  if (digitalRead(BTN_CAL) == LOW)
  {
    Serial.println(">> Botón presionado. Deteniendo robot para calibrar...");

    stopRobotAndTasks(); // apaga motores + suspende tareas
    delay(50);           // pequeño debounce / asegurar suspensión

    Serial.println(">> Iniciando calibración MPU...");
    calibrateMPU(); // hace la calibración (usa delay internamente)
    Serial.println(">> Calibración MPU completa y guardada.");

    delay(50);

    resumeRobotAndTasks(); // reactiva tareas y motores
    Serial.println(">> Robot reanudado.");

    delay(500); // anti-rebote para no volver a entrar inmediatamente
  }
}