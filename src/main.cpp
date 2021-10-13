#include <RBE1001Lib.h>
#include "BlueMotor.h"
#include "Timer.h"
#include "IRdecoder.h"
#include "RemoteConstants.h"
#include "ESP32AnalogRead.h"
//SWANBOT SWANBOT SWANBOT SWANBOT SWANBOT
BlueMotor blueMotor;
ESP32AnalogRead leftLine;
ESP32AnalogRead rightLine;

// things needed to print
long timeToPrint = 0;
long now = 0;
long newPosition = 0;
long oldPosition = 0;
long sampleTime = 100;
int speedInRPM = 0;
int CPR = 270;

// used to move arm for testing purposes
int motorEffort = -1;
int bringDown = -250;
int bringUp = 250;
float encoderCount = 0.0;
float changed = 0.0;

// PID variables
float PIDEffort = 0.0;
float setScaledEffort = 0.0;
const float Kp = 0.35;
const float Ki = 0.0005;
const float Kd = 0.05;
float previousCount = 0.0;
float sumOfErrors = 0.0;

float Fourty5DegreeEncoderCount = 3500.0;
float Twenty5DegreeEncoderCount = 7700.0;
float stagingBlock = 0.0;

Rangefinder ultrasonic;
const float ultraKp = 0.0085;
// roof and block thresholds
int roofApproachThreshold = 18; // how far we want to be from roof to lift arm safely
int roofThreshold = 12;         // how far we want to be from roof to actually pick up/drop off plate
int blockThreshold = 10;        // how far we want to be from block to drop off/pick up

//button
const int buttonPin = BOOT_FLAG_PIN;

//for driving
LeftMotor left_motor;
RightMotor right_motor;
double diam = 2.75;
double track = 5.875;
int defaultSpeed = 130;
double distanceCorrection = 0.95;

const float pivot_diam = 14.2;
const float wheel_diam = 6.8;
const float degreesPerMotor = (pivot_diam * 3.14) / (wheel_diam * 3.14);
const float degreesPerCM = 360.0 / (3.14 * wheel_diam); // cm to degrees formula

//for line following (calling the linefollowing sensor functions)
const int reflectancePin1 = 39;
const int reflectancePin2 = 36;
int threshold = 0.5;
int thresholdHigh = 1250;
double kp = 0.05;
float leftV;
float rightV;

//for servo arm
Servo grip;
const int servoPin = 33;
int openGrip = 140; // find number which make it close
int closeGrip = 50; // find numbers which make it open

//IR remote
IRDecoder decoder(15);
int16_t keyPress;

// boolean variables for robot pick and drop off
bool safeToBeCollected = false; // picking up old collector
bool safeToBeDeposited = false; // depositing old collector
bool safeToPickUpNew = false;   // picking up new collector
bool safeToDropOffNew = false;  // depositing new collector
bool readyToPickUpNew = false;  // once new plate has been placed on block by person

bool depositingNew = false; // false when we pick up old and deposit old, true when we have picked up new plate
int previousState = 0;

//state machines
enum ROBOT_STATES
{
  LINE_FOLLOW_OUT,
  START_SIDE,
  APPROACH_ROOF,
  PICKUP_OLD,
  TOWARD_BLOCK,
  DEPOSIT_OLD,
  WAIT_FOR_SIGNAL,
  PICKUP_NEW,
  BACK_TO_INTERSECTION,
  DEPOSIT_NEW,
  MOVE_OTHER_SIDE,
  IDLE
};

int robotState;

//functions (need to check each of these once sensors are wired)
void lineFollow(float reflectance1, float reflectance2);
void turn(double angle);
double ultrasonicRead();
void straight(double distance);

void pickUpOld();
void pickUpNew();
void depositOld();
void depositNew();

void setup()
{

  ultrasonic.attach(SIDE_ULTRASONIC_TRIG, SIDE_ULTRASONIC_ECHO);

  leftLine.attach(reflectancePin2);
  rightLine.attach(reflectancePin1);

  grip.attach(servoPin);

  blueMotor.setup();

  decoder.init();
  blueMotor.reset();
  robotState = IDLE;
  Serial.begin(9600);
  delay(1000);
}

/*
used to see encoder count as blueMotor spins with positive effort
*/
// void positiveEffort()
// {

//   //Take a screenshot of the effort when the encoder count starts to change.
//   for (float motorEffort = 1.0; motorEffort < 255.0; motorEffort++)
//   {
//     delay(100);
//     blueMotor.setEffort(motorEffort);
//     encoderCount = blueMotor.getPosition();
//     printf("Encoder Count:%f\t Effort:%f\n", encoderCount, motorEffort);
//   }
// }

// /*
// used to see encoder count as blueMotor spins with negative effort

// void negativeEffort()
// {
//   //Take a screenshot of the effort when the encoder count starts to change.
//   for (float motorEffort = -1.0; motorEffort > -255.0; motorEffort--)
//   {
//     delay(100);
//     blueMotor.setEffort(motorEffort);
//     encoderCount = blueMotor.getPosition();
//     printf("Encoder Count:%f\t Effort:%f\n", encoderCount, motorEffort);
//   }
// }

void bringArmDown()
{
  blueMotor.setEffort(bringDown);
}

/*
  function which calls seteffort which scales the given motor effort to adjusted motor effort in positive
*/
// void deadBandPositive()
// {

//   //Take a screenshot of the effort when the encoder count starts to change.
//   for (float motorEffort = 1.0; motorEffort < 255.0; motorEffort++)
//   {
//     now = millis();
//     delay(100);
//     // gives it to setEffortWithoutDB which calculuates new effort
//     changed = blueMotor.setEffortWithoutDB(motorEffort);
//     encoderCount = blueMotor.getPosition();
//     speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
//     oldPosition = blueMotor.getPosition();
//     printf("Time:%ld\t Effort:%f\t Adjusted Effort:%f\t Encoder Count:%f\t Speed(RPM):%d\n", now, motorEffort, changed, encoderCount, speedInRPM);
//   }
// }

// /*
//   function which calls seteffort which scales the given motor effort to adjusted motor effort in negative
// */
// void deadBandNegative()
// {

//   for (float motorEffort = -1.0; motorEffort > -255.0; motorEffort--)
//   {
//     now = millis();
//     delay(100);
//     // gives it to setEffortWithoutDB which calculuates new effort
//     changed = blueMotor.setEffortWithoutDB(motorEffort);
//     encoderCount = blueMotor.getPosition();
//     speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
//     oldPosition = blueMotor.getPosition();
//     printf("Time:%ld\t Effort:%f\t Adjusted Effort:%f\t Encoder Count:%f\t Speed(RPM):%d\n", now, motorEffort, changed, encoderCount, speedInRPM);
//   }
// }

// // takes in the calculated effort from setEffortWithoutDB and prints argument
// void printEverything(float argScaledEffort)
// {
//   long now = millis();
//   encoderCount = blueMotor.getPosition();
//   speedInRPM = ((encoderCount - oldPosition) * 1000 * 60) / (sampleTime * CPR);
//   oldPosition = blueMotor.getPosition();
//   printf("Time:%ld\t Adjusted Effort:%d\n", now, argScaledEffort);
// }

// used to bring motor down to staging block after checking if it reached a certain position
// void comeBackDown()
// {
//   if ((3400.0 < blueMotor.getPosition()) && (blueMotor.getPosition() < 3900.0))
//   {
//     reached45 = true;
//     delay(200);
//   }
//   if (reached45)
//   {
//     PIDEffort = evaluate(blueMotor.getPosition(), stagingBlock);
//     setDBCalculatedEffort = blueMotor.setEffortWithoutDB(PIDEffort);
//     printEverything(setDBCalculatedEffort);
//   }
// }

//line follower which uses the values from the two reflectance pins as arguments
void lineFollow(float reflectance1, float reflectance2)
{
  reflectance1 = leftLine.readVoltage();
  reflectance2 = rightLine.readVoltage();

  float error = reflectance1 - reflectance2;
  float effort = 0;
  effort = kp * error;
  right_motor.setSpeed(defaultSpeed + effort);
  left_motor.setSpeed(defaultSpeed - effort);

  Serial.printf("linetracker: left: %f, right %f\n", reflectance1, reflectance2);
  //delay(300);
}

// turn function which has turn angle as the argument
// positive degrees is left
// negative degrees is right
void turn(double degrees)
{
  left_motor.startMoveFor(-(degrees * degreesPerMotor), 180);
  right_motor.moveFor(degrees * degreesPerMotor, 180);
  delay(1000);
}

// given value is in cm
// positive value is forward
// negative value is backwards
void straight(double cm)
{
  left_motor.startMoveFor(cm * degreesPerCM, 90);
  right_motor.moveFor(cm * degreesPerCM, 90);
}

//read the ultrasonic
double ultrasonicRead()
{
  delay(10);
  double distance = ultrasonic.getDistanceCM();
  //int maxDistance = 160;
  //int minDistance = .9;
  //if (distance > maxDistance) distance = maxDistance;
  //if(distance <= minDistance) distance = 120;
  //add something to take the average if the value is wildly inacurate
  printf("ultrasonic: %f", distance);
  return distance;
}

//timer which is set to 1 / 100th of a second or 10 milliseconds
Timer PIDTimer(10);
float output;

// PID Control Program
float evaluate(long encoderCount, long targetCount)
{
  if (PIDTimer.isExpired())
  {
    printf("enc: %li\n", encoderCount);
    float error = targetCount - encoderCount;
    sumOfErrors += error;
    output = Kp * error - Kd * (encoderCount - previousCount) + Ki * sumOfErrors;
    //printf("Current Encoder Count:%f\t Set Encoder Count:%f\t Output:%f\t", encoderCount, previousCount, output);
    previousCount = encoderCount;
  }
  return output;
}

// used to find encoder counts as motor spins a certain way
void findEncoderCount()
{
  blueMotor.setEffort(-100);
  long answer = blueMotor.getPosition();
  printf("count:%ld\n", answer);
}

// //picking up old plate from roof
// void waitToPickUpOld()
// {

//   if (keyPress == remote1)
//   {
//     safeToBeCollected = true;
//   }
//   else
//   {
//     robotState = IDLE;
//   }
// }

void pickUpOld()
{
  if (safeToBeCollected)
  {
    straight(0.5);

    while (blueMotor.getPosition() < Fourty5DegreeEncoderCount + 4000)
    {
      //printf("reached here, lifting arm more\n");
      PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 4000);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    }

    grip.write(closeGrip);

    while (blueMotor.getPosition() < Fourty5DegreeEncoderCount + 4000)
    {
      //printf("reached here, lifting arm more\n");
      PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 4000);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    }

    straight(1.25);

    while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 12.5)
    {
      //printf("reached here, lifting arm more\n");
      PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 12.5);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    }

    printf("pickup position: %ld\n", blueMotor.getPosition());

    straight(2);
    blueMotor.setEffort(0);

    while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 425)
    {
      //printf("reached here, lifting arm more\n");
      PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 425);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
      blueMotor.setEffort(0);
    }
    // back away from roof small amount
    straight(-3); // have this go backwards

    //forward just a sec once more hoe
    //straight(1);

    // while (blueMotor.getPosition() > Fourty5DegreeEncoderCount - 425)
    // {
    //   //printf("reached here, lifting arm more\n");
    //   PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 425);
    //   setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    // }
    straight(-7);
  }
  else
  {
    printf("Swanbot is waiting to safely collect old plate\n");
  }
}

// depositing old plate on block
void depositOld()
{
  if (safeToBeDeposited)
  {
    // back away from block enough to get back into original postion to be ready to pick up new plate

    // open gripper
    grip.write(openGrip);
    straight(-3);
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    blueMotor.setEffort(0);
    // close gripper
    //grip.write(closeGrip); // CHECK THIS
  }
  else
  {
    printf("Swanbot is waiting to safely deposit old plate\n");
  }
}

// picking up new plate from block
void pickUpNew()
{
  // raise gripper a little above block
  while (blueMotor.getPosition() < blueMotor.getPosition() + 50)
  {
    //printf("reached here, lifting arm more\n");
    PIDEffort = evaluate(blueMotor.getPosition(), blueMotor.getPosition() + 50);
    setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
  }
  //straight(1);
  if (readyToPickUpNew)
  {
    // open gripper
    grip.write(closeGrip);
    while (blueMotor.getPosition() < Fourty5DegreeEncoderCount - 1000)
    {
      //printf("reached here, lifting arm more\n");
      PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 1000);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    }
  }
  else
  {
    printf("Plate is not ready to be picked up by Swanbot\n");
  }
}

// void actuallyPickUp()
// {
//   if (safeToPickUpNew)
//   {
//     // move a little closer to fully grip plate with gripper open
//     straight(5); // have to test to get distance forward

//     // close gripper
//     grip.write(closeGrip);
//   }
//   else
//   {
//     printf("Swanbot is waiting to safely pick up new plate\n");
//   }
// }

// depositing new plate on roof
void depositNew()
{

  // move a little closer to be in line with pins
  straight(2);

  // if button "5" is pressed on the remote
  if (keyPress == remote5)
  {
    safeToDropOffNew = true;
  }
  if (safeToDropOffNew)
  {
    // lower arm a little to put plate on pins
    PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 10); // have to test to find this number
    setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);

    // open gripper to deposit plate
    grip.write(openGrip);

    // raise arm a little to avoid bumping
    PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 10); // have to test to find this number
    setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);

    // back away from roof
    straight(5); // have this go backwards
    grip.write(closeGrip);
  }
  else
  {
    // wait for safeToDropOffNew == true
    printf("Swanbot is waiting safely drop off new plate\n");
  }
}

// state case and switch function
void updateRobotState()
{
  leftV = leftLine.readVoltage();
  rightV = rightLine.readVoltage();

  switch (robotState)
  {
    if (keyPress == remotePlayPause)
    {
      previousState = robotState;
      robotState = IDLE;
    }
  case LINE_FOLLOW_OUT:
    // driving with staging block height
    // PIDEffort = evaluate(blueMotor.getPosition(), stagingBlock);
    // setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);

    if (ultrasonic.getDistanceCM() > roofApproachThreshold)
    {
      //printf("ultrasonic: %f\n", ultrasonic.getDistanceCM());
      lineFollow(leftV, rightV); // keep following line until we approach just before pick up distance
    }
    else
    {
      left_motor.setEffort(0);
      right_motor.setEffort(0);

      robotState = APPROACH_ROOF;
      //printf("done with state");
    }
    break;

  case APPROACH_ROOF:
    if (!depositingNew)
    {
      while (blueMotor.getPosition() < Fourty5DegreeEncoderCount - 220)
      {
        //printf("reached here, lifting arm\n");
        PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount - 220);
        setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort); // move arm to fortyFive roof height
        //delay(200);
      }
    }
    else
    {
      printf("did not reach here\n");
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      // PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 10); // if we are going to deposit plate, want arm to be a little higher than 45 degrees
      // setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);                     // move arm to a little higher than fortyFive roof height
      // delay(200);
    }

    while (ultrasonicRead() > roofThreshold)
    {
      lineFollow(leftV, rightV); //slowly follow line until closest position is reached (right in front of the plate)
      blueMotor.setEffort(0);
    }
    left_motor.setEffort(0);
    right_motor.setEffort(0);

    delay(100);
    if (!depositingNew)
    {
      //printf("reached pick up old state\n");
      grip.write(openGrip);
      blueMotor.setEffort(0);

      // printf("encoder count: %ld", blueMotor.getPosition());
      // printf("i want this count: %ld", blueMotor.getPosition() + 500);

      printf("encoder count: %ld\n", blueMotor.getPosition());

      delay(100);

      while (blueMotor.getPosition() < Fourty5DegreeEncoderCount + 1000)
      {
        //printf("reached here, lifting arm more\n");
        PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount + 1000);
        setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
      }

      delay(500);
      robotState = IDLE;
    }
    else
    {
      robotState = DEPOSIT_NEW; // if depositingNew is true, switch to deposit_new state
    }
    break;

  case PICKUP_OLD: //already at pickup distance with arm raised
    delay(200);
    pickUpOld();
    delay(300);
    turn(175);
    robotState = TOWARD_BLOCK;
    //printf("reached toward block state\n");
    break;

  case IDLE:
    printf("entered Idle\n");
    left_motor.setEffort(0);
    right_motor.setEffort(0);
    blueMotor.setEffort(0);

    if (keyPress == remote1)
    {
      safeToBeCollected = true;
      robotState = PICKUP_OLD;
      //printf("going to pickup old now\n");
    }
    else
    {
      left_motor.setEffort(0);
      right_motor.setEffort(0);
      blueMotor.setEffort(0);
    }

    if (keyPress == remote9)
    {
      robotState = LINE_FOLLOW_OUT;
      printf("starting\n");
    }
    if (keyPress == remote7)
    {
      safeToBeDeposited = true;
      robotState = DEPOSIT_OLD;
    }
    if (keyPress == remote3)
    {
      printf("setting ready to pick up true\n");
      readyToPickUpNew = true;
      robotState = PICKUP_NEW;
    }
    if (keyPress == remotePlayPause)
    {
      robotState = previousState;
    }
    break;
  case TOWARD_BLOCK:
    //straight (-5);
    if ((leftV > 0.3) && (rightV > 0.3))
    { // reached intersection
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(150);
      straight(5);
      turn(82); // turn left to face block
      straight(20);
      left_motor.setSpeed(0);
      right_motor.setSpeed(0);
      delay(1000);
      robotState = DEPOSIT_OLD;
    }
    else
    {
      lineFollow(leftV, rightV);
    }
    break;

  case DEPOSIT_OLD:

    while (blueMotor.getPosition() > stagingBlock + 700)
    {
      printf("dropping off\n");
      PIDEffort = evaluate(blueMotor.getPosition(), stagingBlock + 700);
      setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
    }
    blueMotor.setEffort(0);
    robotState = IDLE;

    depositOld();
    left_motor.setSpeed(0);
    right_motor.setSpeed(0);
    blueMotor.setEffort(0);
    robotState = PICKUP_NEW;

    break;

    // case PICKUP_NEW:
    //   if (!readyToPickUpNew)
    //   {
    //     blueMotor.setEffort(0);
    //     printf("hello we are going to idle\n");
    //     robotState = IDLE;
    //   }
    //   pickUpNew();
    //   delay(2000);
    //   blueMotor.setEffort(0);
    //   robotState = BACK_TO_INTERSECTION;
    //   break;

    // case BACK_TO_INTERSECTION:
    //   blueMotor.setEffort(0);
    //   turn(180);

    //   if ((leftV > .6) && (rightV > .6))
    //   { // reached intersection
    //     left_motor.setSpeed(0);
    //     right_motor.setSpeed(0);
    //     blueMotor.setEffort(0);
    //     delay(150);
    //     turn(-90);            // turn right to face roof
    //     depositingNew = true; // set this to true since we have picked up the new plate going to put back on roof
    //     //robotState = APPROACH_ROOF;
    //     //printf("got to approacj roof");
    //   }
    //   else
    //   {
    //     lineFollow(leftV, rightV);
    //     blueMotor.setEffort(0);
    //   }
    //   break;

    // case DEPOSIT_NEW: // already at pickup distance with arm raised
    //   depositNew();
    //   robotState = MOVE_OTHER_SIDE;
    //   break;

    // case MOVE_OTHER_SIDE:
    //   // bring arm back down to staging position to be set for 25 degree program
    //   PIDEffort = evaluate(blueMotor.getPosition(), stagingBlock);
    //   setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);

    //   // this should bring robot to the intersection facing the roof
    //   turn(-90);    // turn right
    //   straight(15); // go straight
    //   turn(90);     // turn left

    //   // line follow until we find the line
    //   while ((leftV < threshold) && (rightV < threshold))
    //   {
    //     lineFollow(leftV, rightV);
    //   }

    //   turn(90); // turn left

    //   // line follow until we hit intersection
    //   while ((leftV < threshold) && (rightV < threshold))
    //   {
    //     lineFollow(leftV, rightV);
    //   }

    //   turn(90); // turn left to face 25 degree roof
    //   break;
  }
}

// void printLineSensorValues(float reflectance1, float reflectance2)
// {
//   reflectance1 = leftLine.readVoltage();
//   reflectance2 = rightLine.readVoltage();
//   Serial.printf("linetracker: left: %f, right %f\n", reflectance1, reflectance2);
//   //delay(500);
// }

void loop()
{
  // LINE FOLLOW
  // if (leftLine.readVoltage() > 0.6 && rightLine.readVoltage() > 0.6) // stop at intersection
  // {

  //   left_motor.setSpeed(0);
  //   right_motor.setSpeed(0);
  //   Serial.printf("linetracker: left: %f, right %f\n", leftLine.readVoltage(), rightLine.readVoltage());
  // }
  // else
  // {
  //   lineFollow(leftV, rightV);
  // }

  // PRINT LINE SENSOR VALUES
  //printValues(leftV, rightV);

  // TURN
  //turn(90);

  // ULTRASONIC READ
  // if (ultrasonicRead() > 11)
  // {
  //   left_motor.setSpeed(60);
  //   right_motor.setSpeed(60);
  // }
  // else
  // {
  //   left_motor.setSpeed(0);
  //   right_motor.setSpeed(0);
  // }

  // STRAIGHT
  // straight(11);

  // GRIPPER
  // grip.write(openGrip);
  // delay(100);
  // grip.write(closeGrip);
  // delay(100);

  // raise arm to a certain height
  //PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount); // have to test to find this number
  //setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
  bringArmDown();

  //ACTUAL PROGRAM
  // while (true)
  // {
  //   leftV = leftLine.readVoltage();
  //   rightV = rightLine.readVoltage();
  //   keyPress = decoder.getKeyCode();
  //   updateRobotState();
  // }

  // findEncoderCount();

  // if(firstTime) {
  //   PIDEffort = evaluate(blueMotor.getPosition(), Fourty5DegreeEncoderCount);
  //   setScaledEffort = blueMotor.setEffortWithoutDB(PIDEffort);
  //   firstTime = false;
  // }
  // comeBackDown();
}