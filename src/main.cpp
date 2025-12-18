#include <Arduino.h>
#include "Neural_Networks_FF.h"
#include <Wire.h>
#include <MPU6050.h>
#include <driver/ledc.h>
#include <PID_v1.h>
#include <Preferences.h>

#include "config.h"
#include "encoders.h"
#include "motors.h"
#include "mpu_block.h"
#include "nn_cascade_block.h"
#include "line_follower_block.h"
#include "tasks_block.h"

void setup()
{
  Serial.begin(115200);
  Wire.begin(41, 42);
  mpu.initialize();

  pinMode(BTN_CAL, INPUT_PULLUP);

  if (loadCalibration())
  {
    Serial.println(">> Calibración cargada de memoria.");
  }
  else
  {
    Serial.println(">> No existe calibración guardada. Necesitas presionar el botón.");
  }

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

  initNeural();

  for (int i = 0; i < NUM_SENSORS; i++)
    pinMode(sensorPins[i], INPUT);

  if (loadCalibration())
  {
    int16_t ax_init, ay_init, az_init;
    mpu.getAcceleration(&ax_init, &ay_init, &az_init);
    angleFiltered = atan2(
                        (ay_init - ay_offset) / 16384.0,
                        (az_init - az_offset) / 16384.0) *
                    180.0 / PI;
  }

  xTaskCreatePinnedToCore(TaskBalanceo, "TaskBalanceo", 10000, NULL, 3, &taskBalHandle, 1);
  xTaskCreatePinnedToCore(TaskLinea, "TaskLinea", 4000, NULL, 1, &taskLineHandle, 0);

  Serial.println("Robot listo. Presiona el botón para calibrar la MPU.");
}

void loop()
{
  if (digitalRead(BTN_CAL) == LOW)
  {
    Serial.println(">> Botón presionado. Deteniendo robot para calibrar...");

    stopRobotAndTasks();
    delay(50);

    Serial.println(">> Iniciando calibración MPU...");
    calibrateMPU();
    Serial.println(">> Calibración MPU completa y guardada.");

    delay(50);

    resumeRobotAndTasks();
    Serial.println(">> Robot reanudado.");

    delay(500);
  }
}
