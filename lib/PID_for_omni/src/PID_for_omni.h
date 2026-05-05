#ifndef PID_for_omni_h
#define PID_for_omni_h
#include <Arduino.h>

class PID {
  public:
  unsigned long current_pid_motor;
  unsigned long current_pid_imu;
  float pid_motor(float kp, float ki, float kd, float feedback, float setpoint);
  float pid_imu(float kp, float ki, float kd, float feedback, float setpoint);
  void log_value();
  float _output;


  private:
  // Motor PID variables
  float _Error_motor;
  float _LastError_motor;
  float _Integral_motor;
  float _kP_motor;
  float _kI_motor;
  float _kD_motor;

  // IMU PID variables
  float _Error_imu;
  float _LastError_imu;
  float _Integral_imu;
  float _kP_imu;
  float _kI_imu;
  float _kD_imu;
  
  
};


#endif