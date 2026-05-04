#include "PID_for_omni.h"

float PID::pid_motor(float kp, float ki, float kd, float value, float setpoint) {
  current_pid = millis();
  _Error = setpoint - value;
  // Serial.println(_Error);
  _Integral += _Error;
  _Integral = constrain(_Integral, -255.0f, 255.0f);  //anti-windup

  _kP = kp * _Error;
  _kI = ki * _Integral;
  _kD = kd * (_Error - _LastError);


  _output = _kP + _kI + _kD;
  _LastError = _Error;

  // Serial.print(" P:");
  // Serial.print(_kP);
  // Serial.print(" I:");
  // Serial.print(_kI);
  // Serial.print(" D:");
  // Serial.print(_kD);
  // Serial.print(" Error:");
  // Serial.print(_Error);
  // Serial.print(" OUT:");
  // Serial.println(_output);

  // Serial.println(_output);
  return constrain(_output, -255, 255);
}