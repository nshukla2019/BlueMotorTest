#include "BlueMotor.h"
#include <RBE1001Lib.h>


long count = 0;  // encoder counter
// Mutex for the count critical variable
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
const int X = 5;
int errorCount = 0;
int oldValue = 0;



int encoderArray[4][4] = {
  {0, -1, 1, X},
  {1, 0, X, -1},
  {-1, X, 0, 1},
  {X, 1, -1, 0}
};


/**
 * Interrupt service routine (ISR) for both of the encoder
 * channels
 */
void IRAM_ATTR isr() {
  int newValue = digitalRead(19) << 1 | digitalRead(18);
  int value = encoderArray[oldValue][newValue];

  if(value == X) {
    errorCount++;
  }
  else {
    count += value;
  }
  oldValue = newValue;
}

/**
 * Set up all the variables for the blue motor
 * This is not the same as the ESP32 setup() function, although
 * that would be a good place to call this setup function
 */
void BlueMotor::setup() {
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, HIGH);
  pinMode(PWM, OUTPUT);
  attachInterrupt(ENCA, isr, CHANGE);
  attachInterrupt(ENCB, isr, CHANGE);
}

/**
 * Get the current encoder count
 * Returns the encoder count value
 */
long BlueMotor::getPosition() {
  long value;
  portENTER_CRITICAL(&mux);
  value = count;
  portEXIT_CRITICAL(&mux);
  return value;
}

/**
 * Reset the encoder count to zero
 */
void BlueMotor::reset() {
  portENTER_CRITICAL(&mux);
  count = 0;
  portEXIT_CRITICAL(&mux);
}

/**
 * Set the motor effort
 * effort value ranges from -255 to +255
 * Negative values turn one way, positive values turn the other way
 */
void BlueMotor::setEffort(int effort) {
  if (effort < 0) {
    setEffort(-effort, true);
  } else {
    setEffort(effort, false);
  }
}

/**
 * Set the motor effort
 * effort values range from 0-255
 * clockwise is true for one direction, false for the other
 */
void BlueMotor::setEffort(int effort, bool clockwise) {
  if (clockwise) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
}


/**
 * Set the motor effort 0-255
 * clockwise is true for one direction, false for the other
 */
void BlueMotor::setEffortWithoutDB(int effort, bool clockwise) {
  if (clockwise) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
  }
  int value = constrain(effort, 0, 255);
  analogWrite(PWM, value);
}

/**
 * Set the motor effort after calculating scaled motor effort 
 * effort value ranges from -255 to +255
 * Negative values turn one way, positive values turn the other way
 */
float BlueMotor::setEffortWithoutDB(int effort) {
  float adjustedMotorEffort = 0;
  if (effort < 0) {
    adjustedMotorEffort = ((effort/255.0)*195.0)-60.0;
    setEffortWithoutDB(-adjustedMotorEffort, true);
  } 
  else {
    adjustedMotorEffort = ((effort/255.0)*198.0)+57.0;
    setEffortWithoutDB(adjustedMotorEffort, false);
  }
  return adjustedMotorEffort;
  
}


