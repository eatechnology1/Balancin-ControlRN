
/*
  line_follower_block.h - Documentación

  Propósito
  ---------
  Implementa la lógica de seguimiento de línea basada en un arreglo de sensores digitales
  y un controlador PID (biblioteca PID_v1). Calcula la posición de la línea a partir
  de los sensores, alimenta esa lectura a un PID y expone un factor de giro (factor_giro)
  que puede ser usado por el controlador de motores para corregir la trayectoria.

  Variables globales (definidas/externas)
  --------------------------------------
  - volatile float factor_giro
      Factor de corrección calculado por el PID. Rango limitado por pidLinea.SetOutputLimits
      (inicialmente -0.40 .. 0.40). Marcada volatile porque se modifica en la tarea y se
      lee desde otras partes del programa/otro contexto.

  - double lineInput
      Entrada al PID: posición promediada de la línea (valor escalado). Se calcula como
      pos / suma (ver método de lectura de sensores).

  - double lineOutput
      Salida del PID, copiada a factor_giro.

  - double lineSetpoint
      Setpoint del PID: posición objetivo de la línea. Valor inicial 350, que representa
      el centro ideal en la escala usada (0..700).

  - double Kp_line, Ki_line, Kd_line
      Parámetros de lazo PID (ganancia proporcional, integral y derivativa). Valores
      iniciales: Kp = 30, Ki = 0.001, Kd = 10.

  - PID pidLinea
      Instancia del controlador PID (biblioteca PID_v1), inicializada con
      (&lineInput, &lineOutput, &lineSetpoint, Kp, Ki, Kd, DIRECT).

  Dependencias externas (no definidas en este fichero)
  ----------------------------------------------------
  - NUM_SENSORS : número de sensores en el arreglo.
  - sensorPins[] : array de pines digitales asociados a los sensores.
  - Arduino core (digitalRead, millis, Serial).
  - FreeRTOS (vTaskDelay, pdMS_TO_TICKS) para la tarea.
  - PID_v1 library.

  Lógica de lectura de sensores
  ----------------------------
  - Cada sensor es leído con digitalRead(sensorPins[i]).
    Se interpreta como activo LOW: si digitalRead == LOW => val = 1 (línea detectada)
    en caso contrario val = 0.
  - Se calcula:
      suma = suma de val (número de sensores que ven la línea)
      pos  = suma de (val * i * 100)  -> posición escalada por 100 por sensor
      num_blancos = conteo de sensores en estado "blanco" (val == 0)
    Si suma > 0, lineInput = pos / suma (promedio ponderado de índices de sensores),
    luego se llama a pidLinea.Compute() y se asigna factor_giro = lineOutput.
    Si suma == 0 (línea perdida) se asigna factor_giro = 0.

  Escalas y setpoint
  -------------------
  - El cálculo de posición escala cada sensor con un factor 100 por índice (i * 100),
    por eso el setpoint por defecto es 350 (centro esperado en una escala aprox. 0..700).
  - Ajustar NUM_SENSORS o la escala/centro requiere actualizar lineSetpoint en consecuencia.

  Configuración del PID dentro de la tarea
  ----------------------------------------
  - pidLinea.SetMode(AUTOMATIC);
  - pidLinea.SetOutputLimits(-0.40, 0.40);  // limita factor_giro
  - pidLinea.SetSampleTime(100);            // tiempo de muestreo en ms (100 ms)

  Tarea RTOS: TaskLinea
  ---------------------
  - Es una tarea infinita (while true) diseñada para ejecutarse periódicamente.
  - Bucle principal:
      1) Leer sensores y calcular lineInput.
      2) Si hay lectura válida, ejecutar pidLinea.Compute() y actualizar factor_giro.
      3) Si no hay lectura válida, fijar factor_giro = 0.
      4) Cada 200 ms imprime por Serial: posición, salida PID y factor_giro.
      5) Espera vTaskDelay(pdMS_TO_TICKS(100)) — la tarea duerme ~100 ms entre iteraciones.
  - Notas:
      * El SetSampleTime(100) y el vTaskDelay(100 ms) hacen que el PID tenga un muestreo
        coherente con la frecuencia de la tarea (aprox. 10 Hz).
      * El uso de variables volátiles (factor_giro) permite lectura segura desde IRQ/otra tarea,
        pero no sincroniza lecturas de múltiples variables compuestas; usar mecanismos
        de protección si se requiere atomicidad entre lineInput/lineOutput/factor_giro.

  Salida de depuración (Serial)
  -----------------------------
  - Cada 200 ms se imprime:
      "LINE POS: <lineInput> | PID OUT: <lineOutput> | FACTOR GIRO: <factor_giro>"
    útil para ajuste de PID y diagnóstico en tiempo real.

  Condiciones especiales y recomendaciones
  ----------------------------------------
  - Sensores activos LOW: si tus sensores devuelven la lógica inversa, invertir la condición.
  - Cuando no se detecta la línea (suma == 0) actualmente se fija factor_giro = 0;
    puede preferirse una estrategia de recuperación (p. ej. mantener última dirección
    o girar hacia el último lado detectado).
  - Afinar Kp/Ki/Kd y los límites de salida según la dinámica del chasis y actuadores.
  - Si otros contextos leen/escriben variables compartidas (p. ej. lineInput/lineOutput),
    considerar uso de semáforos o mutex para evitar condiciones de carrera.
  - Si se desea mayor resolución, cambiar la escala (i*100) o usar sensores analógicos.

  Licencia
  --------
  - Documentación genérica. Ajustar y comentar acorde a licencias del proyecto.
*/
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
