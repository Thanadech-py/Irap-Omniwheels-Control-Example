#ifndef HOLONOMIC_H
#define HOLONOMIC_H
#include <Arduino.h>
#include <math.h>

//Library for IMU
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

//My own Library
#include "PID_for_omni.h"
#include "DriveMotor.h"


class Holonomic {
  public:

  void holonomic_begin();
  float holonomic_IMU();
  void holonomic_stop();
  void holonomic_drive(float VR, float Alpha, float W);
  
    

  private:
  unsigned long current_IMU_loop;
  float _theta1, _theta2 ,_theta3, _Alpha, _W;

  float _Wheel_Length = 90;
  float _V_Robot;
  float _V_Wheels[3]; 

  float _OrientX, _yaw, _yaw_feedback;
  
  
};


#endif