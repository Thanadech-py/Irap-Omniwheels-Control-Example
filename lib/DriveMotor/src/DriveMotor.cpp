#include "DriveMotor.h"

Motor::Motor(uint8_t idx) {
  _idx = idx;

  if (_idx == 1) {
    _IN1 = MOTOR_M1_IN1;
    _IN2 = MOTOR_M1_IN2;
    _ENA = MOTOR_M1_ENA;
    _ENCA = ENCODER_M1_A;
    _ENCB = ENCODER_M1_B;
    _valid_M = true;
    _valid_E = true;
  } else if (_idx == 2) {
    _IN1 = MOTOR_M2_IN1;
    _IN2 = MOTOR_M2_IN2;
    _ENA = MOTOR_M2_ENA;
    _ENCA = ENCODER_M2_A;
    _ENCB = ENCODER_M2_B;
    _valid_M = true;
    _valid_E = true;
  } else if (_idx == 3) {
    _IN1 = MOTOR_M3_IN1;
    _IN2 = MOTOR_M3_IN2;
    _ENA = MOTOR_M3_ENA;
    _ENCA = ENCODER_M3_A;
    _ENCB = ENCODER_M3_B;
    _valid_M = true;
    _valid_E = true;
  } else {
    _valid_M = false;
    _valid_E = false;
  }
}

Motor* Motor::_enc_idx[3] = { nullptr, nullptr, nullptr };

void Motor::_ENC_M1() {
  if (_enc_idx[0]) _enc_idx[0]->_read_rpm();
}
void Motor::_ENC_M2() {
  if (_enc_idx[1]) _enc_idx[1]->_read_rpm();
}
void Motor::_ENC_M3() {
  if (_enc_idx[2]) _enc_idx[2]->_read_rpm();
}

void Motor::set_power(int16_t pwm) {
  if (pwm < 0) {
    digitalWrite(_IN1, HIGH);
    digitalWrite(_IN2, LOW);
  } else {
    digitalWrite(_IN1, LOW);
    digitalWrite(_IN2, HIGH);
  }
  analogWrite(_ENA, abs(pwm));
}

void Motor::_read_rpm() {
  _currentTime_RISING = micros();
  _currentTime_Motor = millis();

  int direction_motor = 1;
  if (digitalRead(_ENCB) == HIGH) {
    _encoder_count++;
    direction_motor = 1;
  } else {
    _encoder_count--;
    direction_motor = -1;
  }

  _deltaT = (_currentTime_RISING - _previousTime_RISING);
  _freq = 1 / (_deltaT / 1e6);
  _rpm = ((_freq / _PPR) * 60) * direction_motor;
  _previousTime_RISING = _currentTime_RISING;
}

float Motor::get_rpm() {
  if (millis() - _currentTime_Motor > 10) {
    _encoder_count = 0;
    _freq = 0;
    _deltaT = 0;
    _rpm = 0;
  }
  return _rpm;
}

void Motor::set_pid(float kp, float ki, float kd) {
  _KP = kp;
  _KI = ki;
  _KD = kd;
}


void Motor::set_rpm(float setpoint_rpm) {
  if (millis() - _pid.current_pid > 10) {
    float output = _pid.pid_motor(_KP, _KI, _KD, get_rpm(), setpoint_rpm);
    set_power(output);
    _pid.current_pid = millis();
  }
}

void Motor::begin() {
  if (!_valid_M && !_valid_E) return;
  pinMode(_IN1, OUTPUT);
  pinMode(_IN2, OUTPUT);
  pinMode(_ENA, OUTPUT);

  pinMode(_ENCA, INPUT_PULLUP);
  pinMode(_ENCB, INPUT_PULLUP);

  _enc_idx[_idx - 1] = this;

  if (_idx == 1) attachInterrupt(digitalPinToInterrupt(_ENCA), _ENC_M1, RISING);
  else if (_idx == 2) attachInterrupt(digitalPinToInterrupt(_ENCA), _ENC_M2, RISING);
  else if (_idx == 3) attachInterrupt(digitalPinToInterrupt(_ENCA), _ENC_M3, RISING);
}