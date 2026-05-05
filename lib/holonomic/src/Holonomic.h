#ifndef HOLONOMIC_H
#define HOLONOMIC_H
#include <Arduino.h>
#include <math.h>
#include "PID_for_omni.h"
#include "DriveMotor.h"


class Holonomic {
  public:

  void holonomic_begin();
  void holonomic_stop();
  void holonomic_drive(float VR, float Alpha, float W);
  
    

  private:
  float _theta1, _theta2 ,_theta3, _Alpha, _W;

  float _Wheel_Length = 90;
  float _V_Robot;
  float _V_Wheels[3];  
};


#endif