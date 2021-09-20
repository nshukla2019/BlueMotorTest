#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include <button.h>

BlueMotor blueMotor;
Button bootButton(BOOT_FLAG_PIN);


// constexpr int PWMA = 5;
// constexpr int AIN1 = 27;
// constexpr int AIN2 = 23;

long timeToPrint = millis();
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;
int motorEffort = 255;


void setup() {
  Serial.begin(9600);
  blueMotor.setup();
  bootButton.Init();
  Serial.println("Starting...");
}


void loop()
{
    oldPosition = blueMotor.getPosition();
   
  //while (bootButton.CheckButtonPress()){
    // The button is currently pressed.
    blueMotor.setEffort(motorEffort);
    
    now = millis();
    
    if (now > timeToPrint)
    {
      timeToPrint = millis() + sampleTime;
      newPosition = blueMotor.getPosition();
 
      Serial.print("new pos      ");
      Serial.print(newPosition);
      Serial.print("      ");

      Serial.print("old pos      ");
      Serial.print(oldPosition);
      Serial.print("      ");

      long pos = newPosition-oldPosition;
      Serial.print("pos      ");
      Serial.println(pos);

      speedInRPM = ((newPosition-oldPosition)*1000*60)/(sampleTime*CPR);

     

      // Serial.println("new line");
      // Serial.print(now);
      // Serial.print("      ");
      // Serial.print(newPosition);
      // Serial.print("      ");
      // Serial.println(speedInRPM);
      oldPosition = newPosition;
    }
    
  //}

 //blueMotor.setEffort(0);


}

