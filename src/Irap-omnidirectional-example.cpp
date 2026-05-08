#include <Arduino.h>
#include "DriveMotor.h"
#include "Holonomic.h"

// Motor M1(1), M2(2), M3(3);
Holonomic omni;
PID pid;

float degree = 0.0f;
void setup() {
  omni.holonomic_begin();
  // M1.begin();
  // M2.begin();
  // M3.begin();

  // M1.set_pid(0.8, 0.6, 0.1);
  // M2.set_pid(1.2, 1.3, 0.1);
  // M3.set_pid(0.95, 0.9, 0.1);

}

void loop() {
  // M1.set_rpm(200);
  // M2.set_rpm(200);
  // M3.set_rpm(200);
  omni.holonomic_drive(40, 90, 200);
}