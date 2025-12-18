
/*
  Archivo: nn_cascade_block.h
  Propósito:
    Implementa variables globales y la lógica de un controlador en cascada que combina
    una red neuronal feed-forward para ajustar dinámicamente la ganancia proporcional
    del lazo de ángulo y un controlador PI/PD para la regulación de ángulo y velocidad.
    Diseñado para ejecutarse en un ESP32 con entorno Arduino.

  Requisitos / Precondiciones:
    - Llamar a initNeural() una vez antes de usar cascada(...).
    - dt (delta de tiempo) debe ser mayor que 0 al llamar a cascada.
    - La red neuronal y las estructuras Dynamic_Array deben estar correctamente inicializadas.

  Variables globales principales (resumen):
    - net : Neural_Networks_FF
        Instancia de la red neuronal utilizada para ajustar Kp del lazo de ángulo.
    - XI (Dynamic_Array)
        Vector de entrada para la red neuronal. Contiene:
          XI[0] = error de ángulo (setpoint_angle - angle)
          XI[1] = diferencia de error: error - error_1
          XI[2] = diferencia de error anterior: error_1 - error_2
        Antes de entrenar/evaluar se normaliza con factor 100.
    - D (Dynamic_Array)
        Vector objetivo para la red neuronal. Se escribe con setPoint/100.
    - posicion (Dynamic_Array)
        Contiene el ángulo medido escalado (angle/100) y se usa en el entrenamiento online.
    - FAC_STR, EST (Dynamic_Array/char arrays)
        Configuración interna para construir la topología/funciones de activación de la red.
    - Parámetros flotantes de control:
        - setpoint_angle : referencia de ángulo para el lazo de ángulo.
        - setPoint : referencia usada por la red neuronal (escalada internamente).
        - Kp_angle, Ki_angle, Kd_angle : constantes del controlador de ángulo (Kp dinámica).
        - angle_integral, angle_prev_error : estado del integrador y error previo del lazo de ángulo.
        - Kp_speed, Ki_speed : constantes del lazo de velocidad (PI).
        - speed_integral : integrador del lazo de velocidad.
        - MAX_ANGLE : umbral de seguridad; si |angle| > MAX_ANGLE la función devuelve 0.
        - PWM_LIMIT : saturación máxima de salida PWM.

  Funciones exportadas:
    initNeural()
      - Inicializa/crea arrays dinámicos, configura la topología de la red neuronal
        (capas, funciones de activación) y parámetros iniciales (LearningRate, límites).
      - Establece setPoint inicial (por ejemplo 0.700 en código actual).
      - Debe ejecutarse antes de cualquier llamada a cascada().

    cascada(float angle, float rpmLeft, float rpmRight, float dt) -> float
      - Propósito:
          Calcula y devuelve la acción de control (pwm de balanceo) generada por el
          lazo en cascada que combina la salida adaptativa de la red neuronal para Kp
          del controlador de ángulo y un controlador de velocidad PI.
      - Parámetros:
          - angle : ángulo medido (unidades coherentes con setpoint_angle).
          - rpmLeft, rpmRight : velocidades medidas de las ruedas (se usa el promedio).
          - dt : tiempo transcurrido desde la última llamada (segundos).
      - Flujo general:
          1. Calcula error = setpoint_angle - angle.
          2. Actualiza XI con [error, error - error_1, error_1 - error_2] y normaliza por 100.
          3. Escribe D con setPoint/100 y posicion con angle/100.
          4. Llama a net.TRAIN_NET_ONLINE(XI, D, posicion) para entrenamiento online.
          5. Lee la salida de la red neuronal (normalizada) y obtiene Kp_angle = Output * 100.
          6. Calcula la acción PID de ángulo:
               pid_unsat = Kp_angle*error + Ki_angle*angle_integral + Kd_angle*derivative
             con derivative = (error - angle_prev_error)/dt.
          7. Aplica saturación a speed_ref en [-300, 300] y lógica anti-windup para el integrador de ángulo:
             sólo acumula angle_integral si no hay saturación con signo que cause windup.
          8. Calcula error_speed = speed_ref - speed_measured (speed_measured = promedio rpm).
          9. Calcula la señal de PWM no saturada para balanceo:
               pwm_unsat = Kp_speed*error_speed + Ki_speed*speed_integral
             y la saturna en [-PWM_LIMIT, PWM_LIMIT] -> pwm_balanceo_base.
          10. Aplica anti-windup al integrador de velocidad y limita speed_integral en [-150,150].
          11. Si |angle| > MAX_ANGLE: devuelve 0 (corte de seguridad). Si no, devuelve pwm_balanceo_base.
      - Valor devuelto:
          - pwm_balanceo_base (float): señal de control saturada para aplicar a los actuadores.
          - 0 si se supera el umbral de seguridad MAX_ANGLE.
      - Efectos secundarios:
          - Actualiza variables globales: error, error_1, error_2, angle_integral, angle_prev_error,
            speed_integral, Output, Kp_angle y la red neuronal (entrenamiento online).
      - Consideraciones de estabilidad y seguridad:
          - dt debe ser fiable para cálculo correcto de la derivada.
          - El integrador del lazo de velocidad está fuertemente regulado y limitado para evitar acumulaciones excesivas.
          - El control de Kp vía red neuronal puede cambiar dinámicamente la ganancia y por tanto
            afectar la estabilidad; testeo y límites apropiados son recomendables.
          - MAX_ANGLE actúa como freno de seguridad para evitar actuar cuando el sistema cae/está fuera de rango seguro.
      - Notas de implementación:
          - La normalización por 100 se aplica tanto a entradas (XI) como a posicion (angle/100) y a D (setPoint/100),
            por lo que la red trabaja en un rango reducido; por ello la salida se desnormaliza con *100.
          - La función realiza entrenamiento online en cada invocación (net.TRAIN_NET_ONLINE), por lo que
            la frecuencia de llamada y la tasa de aprendizaje afectan comportamiento adaptativo.
          - Revisar valores iniciales de LearningRate y límites en initNeural() según comportamiento observado.

  Sugerencias de uso:
    - Ejecutar initNeural() en la fase de configuración (setup).
    - Llamar a cascada() periódicamente con un dt preciso (por ejemplo, en un bucle con temporización).
    - Supervisar señales de salida y, si es necesario, ajustar límites (MAX_ANGLE, PWM_LIMIT),
      parámetros Ki/Kd y la configuración de la red neuronal para garantizar estabilidad.
*/
#pragma once
#include <Arduino.h>
#include "Neural_Networks_FF.h"
#include "config.h"

// ===================== RN + CONTROL EN CASCADA ======================
extern Neural_Networks_FF net;
extern Dynamic_Array XI, D, FAC_STR, EST, posicion;

extern float vel, error, error_1, error_2, setPoint, Output;
extern float setpoint_angle, Kp_angle, Ki_angle, Kd_angle;
extern float angle_integral, angle_prev_error;
extern float Kp_speed, Ki_speed, speed_integral;
extern float MAX_ANGLE, PWM_LIMIT;

Neural_Networks_FF net = Neural_Networks_FF();
Dynamic_Array XI = Dynamic_Array();
Dynamic_Array D = Dynamic_Array();
Dynamic_Array FAC_STR = Dynamic_Array();
Dynamic_Array EST = Dynamic_Array();
Dynamic_Array posicion = Dynamic_Array();

float vel = 0.00;
float error = 0.00;
float error_1 = 0.00;
float error_2 = 0.00;
float setPoint = 0.000;
float Output = 0.00;

float setpoint_angle = 0.0;
float Kp_angle = 85;
float Ki_angle = 1.0;
float Kd_angle = 1.6;

float angle_integral = 0;
float angle_prev_error = 0;

float Kp_speed = 0.8;
float Ki_speed = 0.000001;
float speed_integral = 0;

float MAX_ANGLE = 40;
float PWM_LIMIT = 255;

inline void initNeural()
{
  XI.NewArray_2D(1, 3);
  XI.ToinitializeArray_2D(0.0);

  D.NewArray_2D(1, 1);
  D.ToinitializeArray_2D(0.0);

  byte XC = XI.GetColumns();
  byte DF = D.GetRows();
  byte XF = XI.GetRows();
  byte L = 3;

  EST.NewArray_2D(1, L);
  EST.ToinitializeArray_2D(0.0);
  float NC = EST.GetColumns();

  float V[] = {XC, 3, DF};
  for (byte i = 0; i < NC; i++)
  {
    EST.WriteArray_2D(0, i, V[i]);
  }

  FAC_STR.CharNewArray_1D(L);
  char *myStrings[] = {"logsig", "logsig", "poslin_lim"};
  for (byte i = 0; i < FAC_STR.GetRows(); i++)
  {
    FAC_STR.CharWriteArray_1D(i, myStrings[i]);
  }

  net.dposlin_limits(1, -1);
  net.LearningRate = 0.00;
  net.FEED_FORWARD_NET(EST, FAC_STR);
  posicion.NewArray_2D(1, 1);
  posicion.ToinitializeArray_2D(0.0);
  net.LearningRate = 2.25;
  setPoint = 0.700;
}

inline float cascada(float angle, float rpmLeft, float rpmRight, float dt)
{
  float derivative = (error - angle_prev_error) / dt;
  posicion.WriteArray_2D(0, 0, angle / 100.00);
  error = setpoint_angle - angle;
  XI.WriteArray_2D(0, 0, error);
  XI.WriteArray_2D(0, 1, error - error_1);
  XI.WriteArray_2D(0, 2, error_1 - error_2);
  net.NORMALIZE_DATA(XI, 100.00);
  D.WriteArray_2D(0, 0, setPoint / 100.00);
  net.TRAIN_NET_ONLINE(XI, D, posicion);
  Output = net.y.ReadArray_2D(net.y.GetRows() - 1, 0);
  error_2 = error_1;
  error_1 = error;
  Kp_angle = Output * 100;

  float pid_unsat = Kp_angle * error + Ki_angle * angle_integral + Kd_angle * derivative;

  float speed_ref = constrain(pid_unsat, -300, 300);
  if (!((pid_unsat != speed_ref) && (error * pid_unsat) > 0))
  {
    angle_integral += error * dt;
  }
  angle_prev_error = error;

  float speed_measured = (rpmLeft + rpmRight) * 0.5;
  float error_speed = speed_ref - speed_measured;

  float pwm_unsat = Kp_speed * error_speed + Ki_speed * speed_integral;
  float pwm_balanceo_base = constrain(pwm_unsat, -PWM_LIMIT, PWM_LIMIT);

  if (!((pwm_unsat != pwm_balanceo_base) && (error_speed * pwm_unsat) > 0))
  {
    speed_integral += error_speed * dt;
  }

  speed_integral = constrain(speed_integral, -150, 150);

  if (abs(angle) > MAX_ANGLE)
    return 0;
  return pwm_balanceo_base;
}
