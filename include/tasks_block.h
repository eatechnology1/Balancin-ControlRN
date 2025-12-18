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
