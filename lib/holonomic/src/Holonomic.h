#ifndef HOLONOMIC_H
#define HOLONOMIC_H

#include <Arduino.h>
#include <math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "PID_for_omni.h"
#include "DriveMotor.h"

class Holonomic {
public:
  struct DriveCommand {
    float VR, Alpha, W;
    unsigned long duration;
  };

  void holonomic_begin();
  void holonomic_stop();
  void update();
  void reset_queue();
  void queue_drive(float VR, float Alpha, float W, unsigned long time);
  // void odom_drive(float x_value, float y_value, float yaw_value);
  void holonomic_drive_raw(float VR, float Alpha, float W);
  void holonomic_ptp(float x_goal, float y_goal, float robot_speed, float W);
  void compute(float value);
  void get_odom();

  static float getYaw() {
    noInterrupts();
    float y = yaw;
    interrupts();
    return y;
  }

  static volatile float yaw;

private:
  // IMU
  unsigned long _current_IMU_loop;
  float _yaw, _yaw_feedback;
  static float holonomic_IMU();
  static float _InverseIMU();

  // Kinematics
  const float _Wheel_Length = 20.0f;
  const float _GOAL_THRESHOLD = 0.05f; // meters
  float _theta1, _theta2, _theta3;
  float _Alpha, _W, _V_Robot;
  float _V_Wheels[3];

  // Odometry
  float _x_previous, _y_previous;
  float _x_current[3], _y_current[3];
  float _x_odom, _y_odom, _yaw_feedback_odom;
  float _x_now,  _y_now;
  float _x_theta_cal, _y_theta_cal;
  float _local_Angle, _local_Displacement, _local_Yaw;
  float _dx, _dy;
  float _direction, _distance;
  

  // Command queue
  static constexpr uint8_t MAX_QUEUE = 10;
  DriveCommand _queue[MAX_QUEUE];
  uint8_t _queue_size  = 0;
  uint8_t _queue_index = 0;
  unsigned long _time_function = 0;
  bool _is_running = false;
};

#endif