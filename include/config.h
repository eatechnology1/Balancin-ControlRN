
/**
 * @file config.h
 * @brief Configuración de pines y parámetros para el proyecto RN_Balancin con ESP32-S3
 *
 * Este archivo declara las constantes y macros usadas para la asignación de pines,
 * parámetros PWM, lecturas de encoders y sensores del seguidor de línea.
 *
 * Secciones principales:
 *  - BOTÓN
 *      BTN_CAL: Pin asignado al botón de calibración.
 *
 *  - PINES MOTORES
 *      AIN1, AIN2, PWMA: Pines de control (dirección y PWM) del motor izquierdo (A).
 *      BIN1, BIN2, PWMB: Pines de control (dirección y PWM) del motor derecho (B).
 *      STBY: Pin de standby / habilitación del puente H.
 *
 *  - PWM
 *      PWM_FREQ: Frecuencia del PWM usada para controlar la potencia de los motores.
 *      PWM_RES: Resolución del PWM en bits.
 *      CH_A, CH_B: Canales LEDC asignados (ESP32) para los PWMA/PWMB.
 *      Nota: LEDC es el controlador PWM del ESP32; usar los canales apropiados al inicializar.
 *
 *  - PINES ENCODERS
 *      R_A, R_B: Pines del encoder del motor derecho (canales A y B).
 *      L_A, L_B: Pines del encoder del motor izquierdo (canales A y B).
 *      CPR: Pulsos por vuelta (Counts Per Revolution) del encoder utilizado.
 *
 *  - SEGUIDOR DE LÍNEA
 *      NUM_SENSORS: Número de sensores del seguidor de línea.
 *      sensorPins[]: Array con los pines digitales analógicos asignados a cada sensor,
 *                    en orden (0..NUM_SENSORS-1).
 *
 * Notas de uso y mantenimiento:
 *  - Verificar compatibilidad de los pines con la variante del ESP32-S3 en la placa.
 *  - Mantener coherencia entre defines (#define) y constantes (const) según el uso
 *    (por ejemplo, valores usados en tiempo de compilación vs. variables tipadas).
 *  - Ajustar PWM_FREQ y PWM_RES según el controlador motor y la respuesta requerida.
 *  - CPR debe reflejar el encoder físico instalado para cálculos de velocidad/posición.
 *  - Los nombres de pines usados aquí se utilizan en la inicialización de GPIO, ISR
 *    de encoders y la lectura de los sensores de línea; documentar cualquier cambio
 *    en el hardware para evitar desajustes en el firmware.
 */
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
