
/**
 * @file tasks_block.h
 * @brief Gestión de tareas y control de bajo nivel para el balanceo y seguimiento de línea.
 *
 * Este fichero declara e implementa funciones de tarea y utilidades para controlar
 * el robot de tipo balanceador sobre ESP32 (S3). Integra la adquisición de sensores,
 * la lógica del controlador en cascada y la conmutación de motores mediante PWM.
 *
 * Requisitos externos:
 *  - Funciones y constantes usadas desde otros módulos:
 *      - updateAngleAndRPM(float dt, float &angle, float &rpmL_f, float &rpmR_f)
 *      - float cascada(float angle, float rpmL, float rpmR, float dt)
 *      - driveMotorsDifferential(float pwmL, float pwmR)
 *      - Constantes/variables: PWM_LIMIT, factor_giro, STBY, CH_A, CH_B, LEDC_LOW_SPEED_MODE
 *  - Librerías de Arduino y FreeRTOS para manejo de tareas y temporización.
 *
 * Objetos globales:
 *  - TaskHandle_t taskBalHandle
 *      Handle de la tarea encargada del balanceo (TaskBalanceo). Se exporta extern para
 *      permitir su manipulación desde otros módulos (suspender/resumir).
 *  - TaskHandle_t taskLineHandle
 *      Handle de la tarea responsable del seguimiento de línea. También exportado extern.
 *
 * Funciones principales:
 *
 *  - TaskBalanceo(void *pvParameters)
 *      @brief Bucle principal de la tarea de balanceo.
 *      @details
 *        - Ejecuta con un periodo nominal de 20 ms (pdMS_TO_TICKS(20)).
 *        - Obtiene ángulo y velocidades de rueda llamando a updateAngleAndRPM con dt=0.020 s.
 *        - Calcula la señal de control principal mediante la función en cascada (cascada).
 *        - Aplica un factor de giro diferencial (factor_giro) para generar PWM independientes
 *          para rueda izquierda y derecha.
 *        - Limita las salidas PWM al rango ±PWM_LIMIT y envía las señales con driveMotorsDifferential.
 *        - Imprime por Serial telemetría básica: ángulo, PWM base, factor de giro y PWMs finales.
 *        - Llama a vTaskDelay para mantener el periodo de la tarea.
 *      @param pvParameters Parámetro de FreeRTOS (no utilizado).
 *
 *  - stopRobotAndTasks()
 *      @brief Detiene los motores y suspende las tareas relacionadas.
 *      @details
 *        - Pone a 0 el duty de los canales PWM configurados y actualiza los duty registers.
 *        - Desactiva el pin STBY (standby) para cortar la alimentación de los motores.
 *        - Si los handles de tarea no son NULL, suspende taskBalHandle y taskLineHandle mediante vTaskSuspend.
 *        - Añade un pequeño delay (20 ms) para asegurar la estabilización de la parada.
 *      @note Diseñada para una parada segura antes de operaciones de mantenimiento o fallo.
 *
 *  - resumeRobotAndTasks()
 *      @brief Reactiva tareas y habilita los motores.
 *      @details
 *        - Si los handles de tarea no son NULL, reanuda taskLineHandle y taskBalHandle via vTaskResume.
 *        - Activa el pin STBY (standby) para permitir que los controladores de motor funcionen.
 *        - Incluye un pequeño delay (20 ms) tras la reactivación.
 *      @note Debe llamarse cuando se quiera retomar la operación normal tras una parada controlada.
 *
 * Consideraciones y buenas prácticas:
 *  - Validar que las dependencias (funciones y constantes externas) estén correctamente definidas
 *    y sincronizadas con la frecuencia de la tarea para evitar aliasing o lecturas inconsistentes.
 *  - Tener en cuenta que Serial.print en la tarea puede afectar la determinismo de la misma;
 *    para despliegues críticos se recomienda usar buffers de telemetría o disminuir la frecuencia
 *    de impresión.
 *  - El uso de vTaskSuspend/vTaskResume es apropiado para control manual, pero en sistemas más
 *    complejos conviene emplear mecanismos de sincronización/eventos para coordinación entre tareas.
 *
 * Lenguaje de documentación: Español.
 */
#pragma once
#include <Arduino.h>
#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "mpu_block.h"
#include "nn_cascade_block.h"
#include "line_follower_block.h"

extern TaskHandle_t taskBalHandle;
extern TaskHandle_t taskLineHandle;

TaskHandle_t taskBalHandle = NULL;
TaskHandle_t taskLineHandle = NULL;

inline void TaskBalanceo(void *pvParameters)
{
  const TickType_t dt_task = pdMS_TO_TICKS(20);

  while (true)
  {
    float dt = 0.020;
    float angle;
    float rpmL_f, rpmR_f;

    updateAngleAndRPM(dt, angle, rpmL_f, rpmR_f);

    float pwm_balanceo_base = cascada(angle, rpmL_f, rpmR_f, dt);

    float factorL = 1.0 + factor_giro;
    float factorR = 1.0 - factor_giro;

    float pwm_finalL = pwm_balanceo_base * factorL;
    float pwm_finalR = pwm_balanceo_base * factorR;

    pwm_finalL = constrain(pwm_finalL, -PWM_LIMIT, PWM_LIMIT);
    pwm_finalR = constrain(pwm_finalR, -PWM_LIMIT, PWM_LIMIT);

    driveMotorsDifferential(pwm_finalL, pwm_finalR);

    Serial.print("Ang: ");
    Serial.print(angle);
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

inline void stopRobotAndTasks()
{
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_A, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_A);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, CH_B, 0);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, CH_B);

  digitalWrite(STBY, LOW);

  if (taskBalHandle != NULL)
    vTaskSuspend(taskBalHandle);
  if (taskLineHandle != NULL)
    vTaskSuspend(taskLineHandle);

  delay(20);
}

inline void resumeRobotAndTasks()
{
  if (taskLineHandle != NULL)
    vTaskResume(taskLineHandle);
  if (taskBalHandle != NULL)
    vTaskResume(taskBalHandle);

  digitalWrite(STBY, HIGH);
  delay(20);
}
