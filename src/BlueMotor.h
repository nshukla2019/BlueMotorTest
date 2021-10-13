#pragma once
#include <RBE1001Lib.h>

class BlueMotor
{
public:
  void setEffort(int effort);
  long getPosition();
  void reset();
  void setup();
  //static portMUX_TYPE mux;
  void setEffort(int effort, bool clockwise);
  void setEffortWithoutDB(int effort, bool clockwise);
  float setEffortWithoutDB(int effort);

private:
  const int PWM = 5;
  const int AIN2 = 23;
  const int AIN1 = 22; //was 27
  const int ENCA = 19;
  const int ENCB = 18;
};
