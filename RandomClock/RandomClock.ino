/**
 * Random Movement Clock controller based on a BKA30D-R5 / VID28-05 dual shaft stepper motor
 * Datasheet at https://cdck-file-uploads-europe1.s3.dualstack.eu-west-1.amazonaws.com/arduino/original/3X/b/0/b0aa1434329fd55ba59f10d853612d71be1a5b07.pdf
 */

// INCLUDES
// AccelStepper library for stepper motor control
// https://www.airspayce.com/mikem/arduino/AccelStepper/
#include <AccelStepper.h>

// CONSTANTS
// The total number of steps required for one complete revolution of the stepper shaft
// From datasheet, in partial step mode each step = 1/3 of a degree, so NUM_STEPS = 360 * 3
constexpr uint16_t NUM_STEPS = 1080;

// GLOBALS
// Motor Control
// When viewed from front and shaft on the right hand side, BKA30D-R5 pins are as follows:
//    ______________
//   / B1 B2  A3 A4 \
//  |          (O)   |
//   \_B4 B3__A2 A1_/
//
// As A2/3 and B2/3 are always driven together, only 6 GPIO pins are needed.
AccelStepper minuteStepper(AccelStepper::HALF3WIRE, D1, D2, D5);
AccelStepper hourStepper(AccelStepper::HALF3WIRE, D4, D6, D7);

void setup() { 
  // Serial interface for debugging only
  Serial.begin(115200);
  Serial.println(__FILE__ __DATE__);

  // Configure each stepper
  hourStepper.setMaxSpeed(200);
  minuteStepper.setMaxSpeed(400);
  hourStepper.setAcceleration(1200);
  minuteStepper.setAcceleration(2000);
}

/**
 * Main Program Loop
 */
void loop() {
	// Set random speed, position and acceleration for each hand
  if(hourStepper.distanceToGo() == 0 ) {
    hourStepper.moveTo(rand() % NUM_STEPS);
    hourStepper.setMaxSpeed((rand() % 2000) + 10);
    hourStepper.setAcceleration((rand() % 200) + 10);
    }
  if(minuteStepper.distanceToGo() == 0) {
    minuteStepper.moveTo(rand() % NUM_STEPS);
    minuteStepper.setMaxSpeed((rand() % 200) + 10);
    minuteStepper.setAcceleration((rand() % 200) + 10);
    }    
  // Call the run() function on both steppers every frame to process any movement
  // required to reach their target position
  hourStepper.run();
  minuteStepper.run();
}