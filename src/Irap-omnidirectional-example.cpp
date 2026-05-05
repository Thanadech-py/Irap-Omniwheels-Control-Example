#include <Arduino.h>
#include "Holonomic.h"
#include "DriveMotor.h"

Holonomic omni;

// Motor M1(1), M2(2), M3(3);
float degree = 0.0f;
void setup() {
  omni.holonomic_begin();

}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read until Enter is pressed
    degree = input.toFloat();                     // Convert string to integer
  }

  omni.holonomic_drive(200, 0, 0);

}