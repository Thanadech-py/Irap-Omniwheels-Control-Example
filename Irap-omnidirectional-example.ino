#include "DriveMotor.h"

Motor M1(1), M2(2), M3(3);

void setup() {
  M1.begin();
  M2.begin();
  M3.begin();

  M1.set_pid(0.1, 0.1, 0.01, 1);
  M2.set_pid(0.1, 0.1, 0.01, 1);
  M3.set_pid(0.1, 0.1, 0.01, 1);
}

void loop() {
  M1.set_rpm(280);
  M2.set_rpm(280);
  M3.set_rpm(280);
}
