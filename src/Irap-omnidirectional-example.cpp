#include <Arduino.h>
#include "Holonomic.h"


Holonomic omni;
PID pid;

float degree = 0.0f;
void setup() {
  omni.holonomic_begin();

}

void loop() {

  

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read until Enter is pressed
    degree = input.toFloat();
  }

  omni.holonomic_drive(60, -90, 45);
}