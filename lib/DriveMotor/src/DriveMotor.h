#ifndef DriveMotor_h
#define DriveMotor_h

#include "Arduino.h"
#include "PID_for_omni.h"


// ============= Motor Driver Pin =============
const uint8_t MOTOR_M1_IN1 = 2;
const uint8_t MOTOR_M1_IN2 = 3;
const uint8_t MOTOR_M1_ENA = 4;

const uint8_t MOTOR_M2_IN1 = 5;
const uint8_t MOTOR_M2_IN2 = 6;
const uint8_t MOTOR_M2_ENA = 7;

const uint8_t MOTOR_M3_IN1 = 8;
const uint8_t MOTOR_M3_IN2 = 9;
const uint8_t MOTOR_M3_ENA = 12;

// ============= Encoder Pin =============
const uint8_t ENCODER_M1_A = 31;
const uint8_t ENCODER_M1_B = 30;

const uint8_t ENCODER_M2_A = 29;
const uint8_t ENCODER_M2_B = 28;

const uint8_t ENCODER_M3_A = 33;
const uint8_t ENCODER_M3_B = 34;


class Motor {
  public:

  Motor(uint8_t idx);

  void begin();
  void set_power(int16_t pwm);
  void set_rpm(float setpoint_rpm);
  float get_rpm();
  void set_pid(float kp, float ki, float kd);

  private:
  //id of each motor
  uint8_t _idx;
  //pin of each motor driver for logic
  uint8_t _IN1, _IN2, _ENA;
  //pin of each motor encoder for logic
  uint8_t _ENCA, _ENCB;
  //value check
  bool _valid_M, _valid_E;
  //encoder ticks
  volatile long _encoder_count;

  static Motor* _enc_idx[3];

  //interupt handler
  PID _pid; //PID object

  //private function
  static void _ENC_M1(); 
  static void _ENC_M2();
  static void _ENC_M3();
  void _read_rpm();
  
  //Encoder PPR
  const int16_t _PPR = 370;

  //Timmer variable
  unsigned long _currentTime_RISING;
  unsigned long _previousTime_RISING = 0;
  unsigned long _currentTime_Motor;

  unsigned long _pid_timmer;

  //float variable
  float _rpm;
  float _freq;
  float _deltaT;
  float _KP;
  float _KI;
  float _KD;

};



#endif 