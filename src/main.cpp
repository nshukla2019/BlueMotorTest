#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include <button.h>

BlueMotor blueMotor;
Button bootButton(BOOT_FLAG_PIN);


// constexpr int PWMA = 5;
// constexpr int AIN1 = 27;
// constexpr int AIN2 = 23;

long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;
int motorEffort = 230;


void setup() {
  Serial.begin(9600);
  blueMotor.setup();
  bootButton.Init();
  delay(1000);
  Serial.println("Starting...");
}


void loop()
{

  delay(1000);

  timeToPrint = millis() + sampleTime;

  oldPosition = blueMotor.getPosition();
  while (bootButton.CheckButtonPress())
  {
    //delay(1000);
    // The button is currently pressed.
    blueMotor.setEffort(0);


    if ((now = millis()) > timeToPrint)
    {
      timeToPrint = now + sampleTime;
      newPosition = blueMotor.getPosition();
      speedInRPM = ((newPosition-oldPosition)*1000*60)/(sampleTime*CPR);
      Serial.println("new line");
      Serial.print(now);
      Serial.print("      ");
      Serial.print(newPosition);
      Serial.print("      ");
      Serial.println(speedInRPM);
      oldPosition = newPosition;
    }
    
  }

 blueMotor.setEffort(motorEffort);
//  Serial.println("POSITION: ");
//  Serial.println(blueMotor.getPosition());
}
