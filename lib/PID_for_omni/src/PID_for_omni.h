#ifndef PID_for_omni_h
#define PID_for_omni_h
/*Library*/
#include <Arduino.h>

class PID {
  public:


  /*Function
      - pid_motor -> Use to calculate the PID value of Motor RPM and control the motor.
      - pid_imu -> Use to calculate the PID value of IMU and control the robot heading.
  */
  float pid_motor(float kp, float ki, float kd, float feedback, float setpoint);
  float pid_imu(float kp, float ki, float kd, float feedback, float setpoint);
  /*Variable for lock loop PID*/
  unsigned long current_pid_motor, current_pid_imu;

  private:
  /*Motor PID variables*/ 
  float _Error_motor, _LastError_motor, _Integral_motor, _kP_motor, _kI_motor, _kD_motor;
  
  /*IMU PID variables*/ 
  float _Error_imu, _LastError_imu, _Integral_imu,  _kP_imu, _kI_imu, _kD_imu;
  /*Output of the PID Calculations*/
  float _output;
};


#endif