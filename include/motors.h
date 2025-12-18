
/**
 * @file motors.h
 * @brief Funciones para configurar y controlar dos motores DC mediante PWM usando LEDC en ESP32.
 *
 * Este módulo proporciona utilidades para:
 *  - Configurar un timer y un canal LEDC para generación de PWM en un pin GPIO.
 *  - Aplicar señales PWM diferenciales a dos motores controlados por un puente H,
 *    gestionando la dirección mediante pares de pines (AIN1/AIN2 y BIN1/BIN2) y el pin de
 *    habilitación/standby (STBY).
 *
 * Requisitos/Dependencias:
 *  - Definiciones en config.h: PWM_FREQ, CH_A, CH_B, STBY, AIN1, AIN2, BIN1, BIN2 u otras
 *    macros/pines utilizados por la aplicación.
 *  - Biblioteca ledc (driver/ledc.h) disponible en el entorno ESP32.
 */

/**
 * @brief Configura un timer y un canal LEDC para generar PWM en un pin.
 *
 * @param pwmPin GPIO donde se emitirá la señal PWM.
 * @param pwmChannel Canal LEDC a utilizar (ledc_channel_t).
 * @param timerNum   Timer LEDC que se asignará al canal (ledc_timer_t).
 *
 * @details
 *  - Configura el timer LEDC en modo de baja velocidad (LEDC_LOW_SPEED_MODE) con
 *    resolución de 8 bits y frecuencia definida por PWM_FREQ.
 *  - Selecciona el reloj automático (LEDC_AUTO_CLK).
 *  - Configura el canal LEDC para el gpio especificado, deshabilita interrupciones
 *    y deja el duty inicial en 0.
 *
 * @note PWM_FREQ debe estar definido en config.h. Esta función sólo prepara el
 *       hardware PWM; no modifica pines de dirección ni activa motores.
 * @note El rango efectivo de duty es 0..255 debido a la resolución de 8 bits.
 */

/**
 * @brief Aplica PWMs diferenciales a los dos motores conectados al puente H.
 *
 * @param pwmL PWM para el motor izquierdo (canal CH_A). El signo determina la dirección:
 *             positivo → adelante, negativo → atrás. La magnitud se interpreta como duty.
 * @param pwmR PWM para el motor derecho (canal CH_B). Misma semántica que pwmL.
 *
 * @details
 *  - Activa el pin STBY (lo pone HIGH) para habilitar los drivers de motor.
 *  - Para cada motor:
 *      - Determina la dirección a partir del signo del PWM y ajusta los pines de dirección
 *        (AIN1/AIN2 para el motor izquierdo; BIN1/BIN2 para el motor derecho).
 *      - Calcula el duty como la magnitud absoluta del PWM acotada a 0..255 y lo aplica
 *        al canal LEDC correspondiente (ledc_set_duty + ledc_update_duty).
 *  - Utiliza resolución de 8 bits (0..255) para el duty.
 *
 * @note Los valores de entrada se tratan como float pero se convierten a enteros en el
 *       rango 0..255 mediante constrain(abs(...), 0, 255).
 * @note La convención de dirección asumida es:
 *         - Motor "adelante": AIN1 LOW, AIN2 HIGH (o equivalente para BIN).
 *         - Motor "atrás":     AIN1 HIGH, AIN2 LOW.
 *
 * @warning Asegúrate de que los pines de dirección (AIN1/AIN2, BIN1/BIN2), el pin STBY
 *          y los canales CH_A/CH_B estén correctamente definidos y cableados antes de usar.
 * @warning No realiza comprobaciones de seguridad adicionales (por ejemplo, límites de corriente).
 */
#pragma once
#include <Arduino.h>
#include <driver/ledc.h>
#include "config.h"

// ===================== PWM MOTOR ===============================

inline void setupMotorPWM(int pwmPin, int pwmChannel, int timerNum)
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
inline void driveMotorsDifferential(float pwmL, float pwmR)
{
  digitalWrite(STBY, HIGH);

  // --- MOTOR IZQUIERDO (CH_A) ---
  int dutyL = constrain(abs(pwmL), 0, 255);

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
