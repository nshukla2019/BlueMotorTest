#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include <button.h>

BlueMotor blueMotor;
Button bootButton(BOOT_FLAG_PIN);

long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;
int motorEffort = -255;


void setup() {
  Serial.begin(9600);
  blueMotor.setup();
  bootButton.Init();
  Serial.println("Starting...");
}


void loop()
{
    timeToPrint = millis() + sampleTime;  
    oldPosition = blueMotor.getPosition();
   
  while (1){
   
    blueMotor.setEffort(motorEffort);
    
    if ((now = millis()) > timeToPrint) // set now to whatever time it is at this point and compare to timetoPrint
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

 blueMotor.setEffort(0);


}

