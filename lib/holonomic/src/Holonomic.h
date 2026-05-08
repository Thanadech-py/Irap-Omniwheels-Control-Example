#ifndef HOLONOMIC_H
#define HOLONOMIC_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "PID_for_omni.h"
#include "DriveMotor.h"

#define MAX_W_SPEED 0.5f  // maximum angular velocity in radians/sec

class Holonomic {
  public:
    void holonomic_begin();
    void holonomic_drive(float VR, float Alpha, float W);
    void holonomic_stop();
    void compute(float value);

    static float getYaw() {
      noInterrupts();
      float y = yaw;
      interrupts();
      return y;
    }

    static volatile float yaw;

  private:
    unsigned long current_IMU_loop;

    float _theta1, _theta2, _theta3;
    float _Alpha, _W;
    float _Wheel_Length = 20.0f;
    float _V_Robot, _yaw;
    float _V_Wheels[3];
    float _yaw_feedback;

    static float holonomic_IMU();
    static float _InverseIMU();
};

#endif