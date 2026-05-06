#include "Holonomic.h"


Motor M1(1), M2(2), M3(3); //Motor object
PID _pid;
Adafruit_BNO055 IMU = Adafruit_BNO055(55, 0x29, &Wire);

void Holonomic::holonomic_begin() {
  Serial.begin(9600);
  M1.begin();
  M2.begin();
  M3.begin();

  M1.set_pid(0.8, 0.6, 0.0);
  M2.set_pid(1.2, 1.3, 0.0);
  M3.set_pid(0.95, 0.9, 0.0);

  /* Initialise the sensor */

  if(!IMU.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("No IMU detected ... Please Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);  // Wait for IMU to stabilize (from example)
}

float Holonomic::holonomic_IMU() {
  sensors_event_t _OrientationData;
  IMU.getEvent(&_OrientationData, Adafruit_BNO055::VECTOR_EULER);
  
  if (_OrientationData.type == SENSOR_TYPE_ORIENTATION) {
    return _OrientationData.orientation.x;
  }
}  


void Holonomic::holonomic_drive(float VR, float Alpha, float W) {
  _Alpha = (Alpha * PI) / 180;
  _theta1 = (90 * PI) / 180;
  _theta2 = (210 * PI) / 180;
  _theta3 = (330 * PI) / 180;
  _yaw = (W * PI) / 180;
  

  if (millis() - current_IMU_loop > 10) {
    _yaw_feedback = (holonomic_IMU() * PI) / 180;
    
    Serial.print("IMU_Value_Rad");
    Serial.print(_yaw_feedback);
    Serial.print(" , ");
    Serial.print("IMU_Val_Angle");
    Serial.print(holonomic_IMU());
    Serial.print(" , ");
    Serial.print("PID Output :");
    Serial.println(_W);

    current_IMU_loop = millis();
  }
  

  if (millis() - _pid.current_pid_imu > 20) {
    _W = _pid.pid_imu(0.3, 0.0, 0.0, _yaw_feedback, _yaw);
    
    _pid.current_pid_imu = millis();
  }

  _V_Wheels[0] = VR * cos(_theta1 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[1] = VR * cos(_theta2 - _Alpha) + (_Wheel_Length * _W);
  _V_Wheels[2] = VR * cos(_theta3 - _Alpha) + (_Wheel_Length * _W);

  M1.set_rpm(_V_Wheels[0]);
  M2.set_rpm(_V_Wheels[1]);
  M3.set_rpm(_V_Wheels[2]);

  // Serial.print("M1_RPM :");
  // Serial.print(M1.get_rpm());
  // Serial.print(" , ");
  // Serial.print("M2_RPM :");
  // Serial.print(M2.get_rpm());
  // Serial.print(" , ");
  // Serial.print("M3_RPM :");
  // Serial.print(M3.get_rpm());

}

void Holonomic::holonomic_stop() {
  M1.set_rpm(0);
  M2.set_rpm(0);
  M3.set_rpm(0);
}