#include <Arduino.h>
#include "DriveMotor.h"
#include "Holonomic.h"

// Motor M1(1), M2(2), M3(3);
Holonomic omni;
PID pid;

float degree = 0.0f;
void setup() {
  omni.holonomic_begin();

  

}

void loop() {
  omni.holonomic_drive(40, 0, 30);
  delay(1000);
  omni.holonomic_stop();
  delay(300);
  omni.holonomic_drive(40, 90, 30);
  delay(1000);
  omni.holonomic_stop();
  delay(300);
  omni.holonomic_drive(40, 180, 30);
  delay(1000);
  omni.holonomic_stop();
  delay(300);
  omni.holonomic_drive(40, 270, 30);
  delay(1000);
  omni.holonomic_stop();
  delay(300);
  omni.holonomic_drive(40, 0, 30);
  delay(1000);
  omni.holonomic_stop();
}