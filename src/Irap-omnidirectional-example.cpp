#include <Arduino.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include "Holonomic.h"


Holonomic omni;
PID pid;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x029, &Wire);

float degree = 0.0f;
void setup() {
  omni.holonomic_begin();
  bno.begin();

}

float IMU_orientation_read(sensors_event_t* event) {
  double x = -1000000;
  if (event->type == SENSOR_TYPE_ORIENTATION) {
  x = event->orientation.x;
  }
  return x;
}

void loop() {
  sensors_event_t orientationData;
  bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);

  // if (Serial.available() > 0) {
  //   String input = Serial.readStringUntil('\n');  // Read until Enter is pressed
  //   degree = input.toFloat();                     // Convert string to integer
  // }

  float yaw = IMU_orientation_read(&orientationData);

  if (millis - pid.current_pid_imu > 20) {
    degree = pid.pid_imu(0.4, 0.0, 0.0, yaw, 45);

    pid.current_pid_imu = millis();
  }

  Serial.print("output");
  Serial.println(degree);

  omni.holonomic_drive(80, 0, degree);

}