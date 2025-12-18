#pragma once
#include <Arduino.h>
#include <PID_v1.h>
#include "config.h"

// ===================== SEGUIDOR DE LÍNEA ======================
extern volatile float factor_giro;
extern double lineInput, lineOutput, lineSetpoint;
extern double Kp_line, Ki_line, Kd_line;
extern PID pidLinea;

volatile float factor_giro = 0.0;

double lineInput = 0;
double lineOutput = 0;
double lineSetpoint = 350; // centro ideal (0..700)

double Kp_line = 30;
double Ki_line = 0.001;
double Kd_line = 10;

PID pidLinea(&lineInput, &lineOutput, &lineSetpoint, Kp_line, Ki_line, Kd_line, DIRECT);

inline void TaskLinea(void *pvParameters)
{
  pidLinea.SetMode(AUTOMATIC);
  pidLinea.SetOutputLimits(-0.40, 0.40);
  pidLinea.SetSampleTime(100);

  while (true)
  {
    int suma = 0;
    int pos = 0;
    int num_blancos = 0;

    for (int i = 0; i < NUM_SENSORS; i++)
    {
      int val = digitalRead(sensorPins[i]) == LOW ? 1 : 0;
      suma += val;
      pos += val * i * 100;
      if (val == 0)
        num_blancos++;
    }

    if (suma > 0)
    {
      lineInput = (double)(pos / suma);
      pidLinea.Compute();
      factor_giro = (float)lineOutput;
    }
    else
    {
      factor_giro = 0;
    }

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
