#include "PID_for_omni.h"

float PID::pid_calculation(float kp, float ki, float kd, float value, float setpoint, float accept_error) {
  current_pid = millis();
  _Error = setpoint - value;
  _Integral += _Error;
  _Derivative = _Error - _LastError;

  _P = kp * _Error;
  _I = ki * _Integral;
  _D = kd * _Derivative;

  _output = _P + _I + _D;

  if (abs(_Error) < accept_error) {
    _LastError = _Error;
    return 0;
  }

  _LastError = _Error;
  return _output;
}
