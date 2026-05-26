#include "Holonomic.h"

// Global objects
Motor M1(1), M2(2), M3(3);
PID _pid;
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x29, &Wire);
IntervalTimer IMU_Timer;

volatile float Holonomic::yaw = 0.0f;

// ─────────────────────────── Setup ───────────────────────────

void Holonomic::holonomic_begin() {
  Serial.begin(115200);

  M1.begin(); M2.begin(); M3.begin();

  M1.set_pid(0.80, 0.60, 0.1);
  M2.set_pid(1.20, 1.30, 0.1);
  M3.set_pid(0.95, 0.90, 0.1);

  if (!IMU.begin()) {
    Serial.print("No IMU detected ... Please Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);
}

// ─────────────────────────── IMU ───────────────────────────

float Holonomic::holonomic_IMU() {
  sensors_event_t _OrientationData;
  IMU.getEvent(&_OrientationData);
  return _OrientationData.orientation.x;
}

float Holonomic::_InverseIMU() {
  static float pYaw = 0.0f;
  float raw      = holonomic_IMU();
  float inverted = fmod(360.0f - raw, 360.0f);
  if (inverted != 0.0f) pYaw = inverted;
  return Holonomic::yaw = pYaw;
}

// ─────────────────────────── Drive ───────────────────────────

void Holonomic::compute(float value) {
  _yaw = value;
  if (millis() - _pid.current_pid_imu > 20) {
    _yaw_feedback          = _InverseIMU();
    _W                     = _pid.pid_imu(3.0, 0.0, 0.0, _yaw_feedback, _yaw);
    _W                     = (_W * PI) / 180;
    _pid.current_pid_imu   = millis();
  }
}

void Holonomic::holonomic_drive_raw(float VR, float Alpha, float W) {
  _Alpha  = (Alpha * PI) / 180;
  _theta1 = ( 90 * PI) / 180;
  _theta2 = (210 * PI) / 180;
  _theta3 = (330 * PI) / 180;
  compute(W);

  _V_Wheels[0] = VR * cos(_theta1 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[1] = VR * cos(_theta2 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[2] = VR * cos(_theta3 - _Alpha) + (_Wheel_Length * _W);

  get_odom();

  M1.set_rpm(_V_Wheels[0]);
  M2.set_rpm(_V_Wheels[1]);
  M3.set_rpm(_V_Wheels[2]);
}

void Holonomic::holonomic_stop() {
  M1.set_power(0);
  M2.set_power(0);
  M3.set_power(0);
}

// ─────────────────────────── Queue ───────────────────────────

void Holonomic::queue_drive(float VR, float Alpha, float W, unsigned long time) {
  if (_queue_size < MAX_QUEUE) {
    _queue[_queue_size++] = {VR, Alpha, W, time};
  }
}

void Holonomic::update() {
  if (_queue_index >= _queue_size) return;

  DriveCommand& cmd = _queue[_queue_index];

  if (!_is_running) {
    _time_function = millis();
    _is_running    = true;
  }

  if (millis() - _time_function < cmd.duration) {
    holonomic_drive_raw(cmd.VR, cmd.Alpha, cmd.W);
  } else {
    holonomic_stop();
    _is_running = false;
    _queue_index++;
  }
}

void Holonomic::reset_queue() {
  _queue_size  = 0;
  _queue_index = 0;
  _is_running  = false;
}

// ─────────────────────────── Odometry ───────────────────────────

void Holonomic::get_odom() {

  static float prev_ticks1 = 0, prev_ticks2 = 0, prev_ticks3 = 0;

  float d1 = M1.get_EncoderTicks - prev_ticks1;
  float d2 = M2.get_EncoderTicks - prev_ticks2;
  float d3 = M3.get_EncoderTicks - prev_ticks3;

  prev_ticks1 = M1.get_EncoderTicks;
  prev_ticks2 = M2.get_EncoderTicks;
  prev_ticks3 = M3.get_EncoderTicks;


  _yaw_feedback_odom = (_yaw_feedback * PI) / 180;
  /*Read X value from Encoder*/
  _x_current[0] = cos(_theta1) * d1;
  _x_current[1] = cos(_theta2) * d2;
  _x_current[2] = cos(_theta3) * d3;
  /*Read Y value from Encoder*/
  _y_current[0] = sin(_theta1) * d1;
  _y_current[1] = sin(_theta2) * d2;
  _y_current[2] = sin(_theta3) * d3;
  /*Convert X-Y value from encoder pules to meter*/
  _x_odom = (_x_current[0] + _x_current[1] + _x_current[2]) / 3100.0f;
  _y_odom = (_y_current[0] + _y_current[1] + _y_current[2]) / 3100.0f;



  /*Referance Moving Frame*/
  _x_theta_cal = ((cos(_yaw_feedback_odom) * _x_odom) - (sin(_yaw_feedback_odom) * _y_odom));
  _y_theta_cal = ((sin(_yaw_feedback_odom) * _x_odom) + (cos(_yaw_feedback_odom) * _y_odom));

  _x_now = _x_previous + _x_theta_cal;
  _y_now = _y_previous + _y_theta_cal;

  _x_previous = _x_now; 
  _y_previous = _y_now;
  
  Serial.print("X Now :");
  Serial.print(_x_now);
  Serial.print(" , ");
  Serial.print("Y Now :");
  Serial.print(_y_now);
  Serial.print(" , ");
  Serial.print("X Odom :");
  Serial.print(_x_odom);
  Serial.print(" , ");
  Serial.print("Y Odom :");
  Serial.println(_y_odom);
}
