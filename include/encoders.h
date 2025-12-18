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
