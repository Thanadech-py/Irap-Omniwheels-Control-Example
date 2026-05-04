#ifndef PID_for_omni_h
#define PID_for_omni_h
#include <Arduino.h>

class PID {
  public:
  unsigned long current_pid;
  float pid_motor(float kp, float ki, float kd, float value, float setpoint);
  void log_value();
  float _output;


  private:
  float _Error;
  float _LastError;
  float _Integral;

  float _kP;
  float _kI;
  float _kD;
  
  
};


#endif