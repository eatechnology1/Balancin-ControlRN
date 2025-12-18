
/**
 * @file encoders.h
 * @brief Contadores e ISRs para encoders por cuadratura (uso con ESP32/Arduino).
 *
 * Este módulo proporciona los contadores globales de pulsos para los encoders
 * izquierdo y derecho, y las rutinas de servicio de interrupción (ISRs) que
 * actualizan dichos contadores en función de las señales de los canales A/B.
 *
 * @note Los contadores se declaran como variables volátiles (volatile) porque
 *       son modificados desde ISRs.
 * @note Las ISRs están marcadas con IRAM_ATTR para garantizar que el código
 *       resida en IRAM y pueda ejecutarse desde la ISR en microcontroladores ESP32.
 * @warning Evitar definir las mismas variables globales en varios archivos:
 *          deben existir una única definición para cada contador.
 */

/**
 * @brief Contador de pulsos del encoder derecho.
 *
 * Representa el recuento incremental/decremental de ticks del encoder derecho.
 * Se incrementa o decrementa desde la ISR correspondiente (isr_RA) según la
 * secuencia de cuadratura leída en los pines R_A y R_B.
 *
 * @note Tipo volatile porque su valor cambia desde una ISR.
 * @warning Si se lee este contador desde el contexto principal y la plataforma
 *          requiere acceso atómico (por ejemplo variables de mayor tamaño que el
 *          bus nativo), encapsular la lectura en una sección crítica o deshabilitar
 *          temporalmente las interrupciones para evitar lectura inconsistente.
 */

/**
 * @brief Contador de pulsos del encoder izquierdo.
 *
 * Representa el recuento incremental/decremental de ticks del encoder izquierdo.
 * Se incrementa o decrementa desde la ISR correspondiente (isr_LA) según la
 * secuencia de cuadratura leída en los pines L_A y L_B.
 *
 * @note Tipo volatile porque su valor cambia desde una ISR.
 * @warning Aplicar las mismas precauciones de acceso atómico que para el contador derecho.
 */

/**
 * @brief ISR para el canal A del encoder derecho.
 *
 * Comportamiento:
 *  - Lee los estados de los pines R_A y R_B.
 *  - Si R_A == R_B, incrementa countR.
 *  - Si R_A != R_B, decrementa countR.
 *
 * @details Diseñada para ser llamada desde una interrupción de cambio del pin
 *          (por ejemplo attachInterrupt(digitalPinToInterrupt(R_A), isr_RA, CHANGE);).
 *
 * @note Marcada con IRAM_ATTR para ejecución segura desde la ISR en ESP32.
 * @warning Mantener la ISR lo más corta y rápida posible: evitar llamadas lentas,
 *          operaciones bloqueantes o asignaciones dinámicas dentro de la ISR.
 */

/**
 * @brief ISR para el canal A del encoder izquierdo.
 *
 * Comportamiento:
 *  - Lee los estados de los pines L_A y L_B.
 *  - Si L_A == L_B, incrementa countL.
 *  - Si L_A != L_B, decrementa countL.
 *
 * @details Diseñada para ser llamada desde una interrupción de cambio del pin
 *          (por ejemplo attachInterrupt(digitalPinToInterrupt(L_A), isr_LA, CHANGE);).
 *
 * @note Marcada con IRAM_ATTR para ejecución segura desde la ISR en ESP32.
 * @warning Mantener la ISR lo más corta y rápida posible: evitar llamadas lentas,
 *          operaciones bloqueantes o asignaciones dinámicas dentro de la ISR.
 */
#pragma once
#include <Arduino.h>
#include "config.h"

// Contadores globales de encoders
extern volatile long countR;
extern volatile long countL;

// Definición (una sola vez, aquí)
volatile long countR = 0;
volatile long countL = 0;

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
