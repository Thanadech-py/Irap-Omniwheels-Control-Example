#include "PID_for_omni.h"

float PID::pid_motor(float kp, float ki, float kd, float feedback, float setpoint) {
  current_pid_motor = millis();
  _Error_motor = setpoint - feedback;
  // Serial.println(_Error_motor);
  _Integral_motor += _Error_motor;
  _Integral_motor = constrain(_Integral_motor, -255.0f, 255.0f);  //anti-windup

  _kP_motor = kp * _Error_motor;
  _kI_motor = ki * _Integral_motor;
  _kD_motor = kd * (_Error_motor - _LastError_motor);


  _output = _kP_motor + _kI_motor + _kD_motor;
  _LastError_motor = _Error_motor;
  return constrain(_output, -255, 255);
}

float PID::pid_imu(float kp, float ki, float kd, float feedback, float setpoint) {
  current_pid_imu = millis();
  _Error_imu = setpoint - feedback;
  _Integral_imu += _Error_imu;
  _Integral_imu = constrain(_Integral_imu, -360, 360);

  _kP_imu = kp * _Error_imu;
  _kI_imu = ki * _Integral_imu;
  _kD_imu = kd * (_Error_imu - _LastError_imu);

  _output = _kP_imu + _kI_imu + _kD_imu;
  _LastError_imu = _Error_imu;
  return constrain(_output, -360, 360);
}