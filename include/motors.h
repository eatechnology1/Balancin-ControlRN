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
