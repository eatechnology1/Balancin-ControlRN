#pragma once
#include <Arduino.h>
#include <driver/ledc.h>

// ================= BOTÓN ===============
#define BTN_CAL 10

// ================= PINES MOTORES ==========================
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
const ledc_channel_t CH_A = LEDC_CHANNEL_0; // Motor Izquierdo (A)
const ledc_channel_t CH_B = LEDC_CHANNEL_1; // Motor Derecho (B)

// ===================== PINES ENCODERS ==========================
#define R_A 3
#define R_B 46
#define L_A 9
#define L_B 11
const int CPR = 44;

// ===================== SEGUIDOR DE LÍNEA ======================
#define NUM_SENSORS 8
const uint8_t sensorPins[NUM_SENSORS] = {48, 45, 0, 35, 36, 37, 38, 40};
