#include "Holonomic.h"


Motor M1(1), M2(2), M3(3);

void Holonomic::holonomic_begin() {
  Serial.begin(9600);
  M1.begin();
  M2.begin();
  M3.begin();

  M1.set_pid(0.9, 0.9, 0.0);
  M2.set_pid(1.2, 1.3, 0.0);
  M3.set_pid(1.2, 0.9, 0.01);
}


void Holonomic::holonomic_drive(float VR, float Alpha, float W) {
  
  _Alpha = (Alpha * PI) / 180;
  _W = (W * PI) / 180;
  _theta1 = (90 * PI) / 180;
  _theta2 = (210 * PI) / 180;
  _theta3 = (330 * PI) / 180;



  _V_Wheels[0] = VR * cos(_theta1 - Alpha) + (_Wheel_Length * _W);
  _V_Wheels[1] = VR * cos(_theta2 - Alpha) + (_Wheel_Length * _W);
  _V_Wheels[2] = VR * cos(_theta3 - Alpha) + (_Wheel_Length * _W);

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

  // Serial.print("  |  ");
  // Serial.print("VW1 :");
  // Serial.print(_V_Wheels[0]);
  // Serial.print(" , ");
  // Serial.print("VW2 :");
  // Serial.print(_V_Wheels[1]);
  // Serial.print(" , ");
  // Serial.print("Vw3 :");
  // Serial.println(_V_Wheels[2]);
  
}

void Holonomic::holonomic_stop() {
  M1.set_rpm(0);
  M2.set_rpm(0);
  M3.set_rpm(0);
}