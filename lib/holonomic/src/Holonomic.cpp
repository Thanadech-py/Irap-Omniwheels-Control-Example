#include "Holonomic.h"

#define MAX_W_SPEED 0.5f  // maximum angular velocity in radians/sec

Motor M1(1), M2(2), M3(3); //Motor object
PID _pid;
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x29, &Wire);
IntervalTimer IMU_Timer;

// float yaw = 0.0f;
volatile float Holonomic::yaw = 0.0f;


void Holonomic::holonomic_begin() {
  Serial.begin(115200);
  M1.begin();
  M2.begin();
  M3.begin();

  M1.set_pid(0.8, 0.6, 0.1);
  M2.set_pid(1.2, 1.3, 0.1);
  M3.set_pid(0.95, 0.9, 0.1);


  /* Initialise the sensor */
  if(!IMU.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No IMU detected ... Please Check your wiring or I2C ADDR!");
    while(1);
  }
  delay(1000);  // Wait for IMU to stabilize (from example)

  // sensors_event_t _OrientationData;
  // IMU_Timer.begin(_InverseIMU, 120000);
}


static float Holonomic::holonomic_IMU() {
  sensors_event_t _OrientationData;
  IMU.getEvent(&_OrientationData);
  return _OrientationData.orientation.x;
} 

float Holonomic::_InverseIMU() {
  static float pYaw = 0.0f;
  float raw = holonomic_IMU();
  float inverted = fmod(360.0f - raw, 360.0f);

  if (inverted != 0.0f) {
    pYaw = inverted;
  }

  return Holonomic::yaw = pYaw;
}

void Holonomic::compute(float value) {
  _yaw = value;
  if (millis() - _pid.current_pid_imu > 20) {
    _yaw_feedback = _InverseIMU();
    _W = _pid.pid_imu(3.0, 0.0, 0.0, _yaw_feedback, _yaw);
    _W = (_W * PI) / 180;
    _pid.current_pid_imu = millis();
  } 
}

// เพิ่มคำสั่งเข้า queue
void Holonomic::queue_drive(float VR, float Alpha, float W, unsigned long time) {
  if (_queue_size < 10) {
    _queue[_queue_size++] = {VR, Alpha, W, time};
  }
}

// เรียกใน loop() อันเดียว แทน holonomic_drive()
void Holonomic::update() {
  // ไม่มีคำสั่งใน queue
  if (_queue_index >= _queue_size) return;

  DriveCommand& cmd = _queue[_queue_index];

  // เริ่มคำสั่งใหม่
  if (!_is_running) {
    _time_function = millis();
    _is_running = true;
    Serial.printf("CMD %d: VR=%.1f Alpha=%.1f\n", _queue_index, cmd.VR, cmd.Alpha);
  }

  // ยังไม่หมดเวลา → ขับปกติ
  if (millis() - _time_function < cmd.duration) {
    holonomic_drive_raw(cmd.VR, cmd.Alpha, cmd.W);
  } 
  // หมดเวลา → ไปคำสั่งถัดไป
  else {
    holonomic_stop();
    _is_running = false;
    _queue_index++;
  }
}

// ฟังก์ชันขับมอเตอร์จริงๆ (ไม่มี timer)
void Holonomic::holonomic_drive_raw(float VR, float Alpha, float W) {
  _Alpha = (Alpha * PI) / 180;
  _theta1 = (90  * PI) / 180;
  _theta2 = (210 * PI) / 180;
  _theta3 = (330 * PI) / 180;
  compute(W);

  _V_Wheels[0] = VR * cos(_theta1 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[1] = VR * cos(_theta2 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[2] = VR * cos(_theta3 - _Alpha) + (_Wheel_Length * _W);

  M1.set_rpm(_V_Wheels[0]);
  M2.set_rpm(_V_Wheels[1]);
  M3.set_rpm(_V_Wheels[2]);
}

void Holonomic::reset_queue() {
  _queue_size  = 0;
  _queue_index = 0;
  _is_running  = false;
}

void Holonomic::holonomic_stop() {
  M1.set_power(0);
  M2.set_power(0);
  M3.set_power(0);
}


