/**
 * Experiment: Controlling A Stepper Motor With Your Muscles
 * Link: https://backyardbrains.com/experiments/MuscleSpikerShield_StepperMotor
 * 
 * Ported By: Ben Antonellis
 * Date: February 4th, 2021
 * Edited By: Miguel Cornejo
 * Date: March 4th, 2021
 * Updated By: Ben Antonellis
 * Date: March 22nd, 2021
**/
#include <AccelStepper.h> //Include AccelStepper library, via Arduino or Internet
#include "NeuroBoard.hpp"

NeuroBoard board;

#define DIR 7                   //DIR pin from A4988 to pin 7
#define STEP 4                  //STEP pin from A4988 to pin 4
#define MOTOR_INTERFACE_TYPE 1  //How many motors are connected (Maximum motors are 4)

AccelStepper stepper = AccelStepper(MOTOR_INTERFACE_TYPE, STEP, DIR);

#define MAX 60   						// Maximum reading possible. PLAY WITH THIS VALUE!
#define MAX_STEPS 10 					// This is the maximum number of steps that will advance (You can modify this value).
int readings[10];  						// Array of readings.
int finalReading;						// Averaged out reading from readings array.
byte multiplier = 1;					// Multiplier for analog readings.
byte numSteps = 0;						// The number of steps to take.
int currentSteps = 0;					// The current number of steps taken.

void setup() {

	board.startMeasurements();
	stepper.setMaxSpeed(200);			// The RPMs engine speed is specified.

}

void loop() {

	for (int i = 0; i < 10; i++) {
		readings[i] = board.getNewSample() * multiplier;
		finalReading += readings[i];
	}
	
	finalReading /= 10;
	finalReading = constrain(finalReading, 0, MAX);
	numSteps = map(finalReading, 0, MAX, 0, MAX_STEPS);

	if (numSteps >= 10) {
		stepper.setSpeed(1000);
		stepper.runSpeed();
	} else {
		for (currentSteps = 0; currentSteps <= numSteps; currentSteps++) {
			Serial.print("Current Step: ");
			Serial.print(currentSteps);
			Serial.print(" Final Reading: ");
			Serial.print(finalReading);
			Serial.print(" Number of steps: ");
			Serial.println(numSteps);
			stepper.stop();
			delay(50);
		}
	}
	
	delay(10);

}