#include <Arduino.h>
#include "DriveMotor.h"
#include "Holonomic.h"
bool _started = false;
// Motor M1(1), M2(2), M3(3);
Holonomic robot;
PID pid;

float degree = 0.0f;
void setup() {
  robot.holonomic_begin();
}

void loop() {
  if (!_started) {
    robot.queue_drive(20,  0,  0, 2000); // เดินหน้า 2s
    robot.queue_drive(20, 90,  0, 2000); // เดินซ้าย 3s
    robot.queue_drive(20,180,  0, 2000); // หมุน 1.5s
    robot.queue_drive(20,270, 0, 2000); // หมุนกลับ 1.5s
    _started = true;
  }

  robot.update(); // ← เรียกอันนี้อันเดียวใน loop()
}