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
}


void loop()
{
  timeToPrint = millis() + sampleTime;
  //Serial.println("Time to Print:" + timeToPrint);

  oldPosition = blueMotor.getPosition();
  while (bootButton.CheckButtonPress())
  {
    // The button is currently pressed.
    blueMotor.setEffort(0);
    //Serial.println("Now: "+ now);
    if ((now = millis()) > timeToPrint)
    {
      timeToPrint = now + sampleTime;
      newPosition = blueMotor.getPosition();
      speedInRPM = ((newPosition-oldPosition)*1000*60)/(sampleTime*CPR);
      Serial.print(now);
      Serial.print("      ");
      Serial.print(newPosition);
      Serial.print("      ");
      Serial.println(speedInRPM);
      oldPosition = newPosition;
    }
    
  }

 blueMotor.setEffort(motorEffort);

}


// void forwards() {
//   digitalWrite(AIN1, HIGH);
//   digitalWrite(AIN2, LOW);
//   digitalWrite(PWMA, HIGH);
// }

// void backwards() {
//   digitalWrite(AIN1, LOW);
//   digitalWrite(AIN2, HIGH);
//   digitalWrite(PWMA, HIGH);
// }

// void stop() { digitalWrite(PWMA, LOW); }

// void loop() {
//   forwards();
//   delay(1000);
//   stop();
//   delay(1000);
//   backwards();
//   delay(1000);
// }

// void loop() {
//   while (!bootButton.CheckButtonPress()) {
//     delay(100);
//   }
//   blueMotor.reset();
//   for (int i = 0; i < 255; i++) {
//     blueMotor.setEffort(255 - i);
//     delay(10);
//   }
//   blueMotor.setEffort(0);
//   delay(1000);
//   printf("%ld\n", blueMotor.getPosition());
// }



