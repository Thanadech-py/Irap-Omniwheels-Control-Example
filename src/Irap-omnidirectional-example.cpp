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

  /*Queue command*/
  /*if (!_started) {
    robot.queue_drive(50,  45,  90, 5000); // เดินหน้า 2s
    // robot.queue_drive(0, 90,  0, 3000); // เดินซ้าย 3s
    // robot.queue_drive(0,180,  90, 3000); // หมุน 1.5s
    // robot.queue_drive(0,270, 0, 3000); // หมุนกลับ 1.5s
    _started = true;
  }

  robot.update();*/

  /*Point to Point Navigation Command*/
  robot.holonomic_ptp(0.5, -0.5, 50, 0);

}