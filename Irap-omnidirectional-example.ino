#include "DriveMotor.h"
#include "PID_for_omni.h"

Motor M1(1), M2(2), M3(3);

float speed_ = 0.0;

void setup() {
  Serial.begin(9600);
  M1.begin();
  M2.begin();
  M3.begin();

  M1.set_pid(0.9, 0.9, 0.0);
  M2.set_pid(1.2, 1.3, 0.0);
  M3.set_pid(1.2, 0.9, 0.01);
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Read until Enter is pressed
    speed_ = input.toFloat(); // Convert string to integer
  }

  M1.set_rpm(0);
  M2.set_rpm(0);
  M3.set_rpm(speed_);

  // M1.set_power(255);
  // M2.set_power(255);
  // M3.set_power(255);

  Serial.print("Setpoint:");
  Serial.print(speed_);
  Serial.print(",");
  Serial.print("M1RPM:");
  Serial.print(M1.get_rpm());
  Serial.print(",");
  Serial.print("M2RPM:");
  Serial.print(M2.get_rpm());
  Serial.print(",");
  Serial.print("M3RPM:");
  Serial.println(M3.get_rpm());
}
