#ifndef DriveMotor_h
#define DriveMotor_h

/*Library*/
#include "Arduino.h"
#include "PID_for_omni.h"

/*============= Motor Driver Pin =============*/ 
const uint8_t MOTOR_M1_IN1 = 2;
const uint8_t MOTOR_M1_IN2 = 3;
const uint8_t MOTOR_M1_ENA = 4;

const uint8_t MOTOR_M2_IN1 = 5;
const uint8_t MOTOR_M2_IN2 = 6;
const uint8_t MOTOR_M2_ENA = 7;

const uint8_t MOTOR_M3_IN1 = 8;
const uint8_t MOTOR_M3_IN2 = 9;
const uint8_t MOTOR_M3_ENA = 12;

/*============= Encoder Pin =============*/
const uint8_t ENCODER_M1_A = 31;
const uint8_t ENCODER_M1_B = 30;

const uint8_t ENCODER_M2_A = 29;
const uint8_t ENCODER_M2_B = 28;

const uint8_t ENCODER_M3_A = 33;
const uint8_t ENCODER_M3_B = 34;

/*============= Encoder PPR =============*/
const int16_t PPR = 370;

class Motor {
  public:

  /*Motor Object*/
  Motor(uint8_t idx);

  /*
  Function
      - begin -> Set each Motor and Encoder PIN via ID of Motor Object
      - set_pid -> Set each Motor PID variable for the calculation
      - set_power -> Control Motor using PWM (Open-loop control)
      - set_rpm -> Control Motor using RPM (Closed-loop control)
      - get_rpm -> To get the RPM feedback from Encoder
  */
  void begin();
  void set_pid(float kp, float ki, float kd);
  void set_power(int16_t pwm);
  void set_rpm(float setpoint_rpm);
  float get_rpm();

  /*Variable that return the value from _encoder_count which is a private variable
    so the user can use the pulse from encoder but can not change the value.*/
  const volatile long& get_EncoderTicks;


  private:

  /*Variable for assign each Motor and Encoder PIN */
  uint8_t _idx, _IN1, _IN2, _ENA, _ENCA, _ENCB;
  /*Variable for Lock loop*/
  unsigned long _currentTime_RISING, _previousTime_RISING, _currentTime_Motor, _pid_timmer;
  /*Variable for RPM Calculation*/
  float _rpm, _deltaT, _freq;
  /*Variable for PID gain*/
  float _KP, _KI, _KD;
  /*Variable for store the pulse that are readed from Encoder*/
  volatile long _encoder_count;

  /*Encoder ID by Motor*/
  static Motor* _enc_idx[3];
  /*PID object*/
  PID _pid; 

  /*private function*/
  void _read_rpm();
  static void _ENC_M1();
  static void _ENC_M2();
  static void _ENC_M3(); 
};
#endif 