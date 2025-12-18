#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <Preferences.h>

// ===================== MPU ====================================
extern MPU6050 mpu;
extern Preferences prefs;
extern float ax_offset, ay_offset, az_offset;
extern float gx_offset;
extern float angleFiltered;

MPU6050 mpu;
Preferences prefs;
float ax_offset = 0, ay_offset = 0, az_offset = 0;
float gx_offset = 0;
float angleFiltered = 0;

inline void saveCalibration()
{
  prefs.begin("mpu", false);
  prefs.putFloat("ax_off", ax_offset);
  prefs.putFloat("ay_off", ay_offset);
  prefs.putFloat("az_off", az_offset);
  prefs.putFloat("gx_off", gx_offset);
  prefs.end();
  Serial.println(">> Calibración guardada.");
}

inline bool loadCalibration()
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

inline void calibrateMPU()
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

inline void updateAngleAndRPM(float dt,
                              float &angle,
                              float &rpmL_f,
                              float &rpmR_f)
{
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

  static float rpmR_f_local = 0, rpmL_f_local = 0;
  rpmR_f_local = 0.7 * rpmR_f_local + 0.3 * rpmR;
  rpmL_f_local = 0.7 * rpmL_f_local + 0.3 * rpmL;

  angle = angleFiltered;
  rpmL_f = rpmL_f_local;
  rpmR_f = rpmR_f_local;
}
