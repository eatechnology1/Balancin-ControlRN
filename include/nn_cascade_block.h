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
