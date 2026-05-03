#ifndef PID_for_omni_h
#define PID_for_omni_h
#include <Arduino.h>

class PID {
  public:
  unsigned long current_pid;
  float pid_calculation(float kp, float ki, float kd, float value, float setpoint, float accept_error);


  private:
  float _Error;
  float _LastError;

  float _Integral;
  float _Derivative;

  float _P;
  float _I;
  float _D;
  
  float _output;
};


#endif