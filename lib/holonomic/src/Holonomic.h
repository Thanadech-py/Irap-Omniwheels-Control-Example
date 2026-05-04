#ifndef HOLONOMIC_H
#define HOLONOMIC_H
#include <Arduino.h>

class Holonomic {
  public:

  float holonomic_drive();
    

  private:
  float _V_Robot;
  float _V_Wheels;
  float _Theta;
};


#endif