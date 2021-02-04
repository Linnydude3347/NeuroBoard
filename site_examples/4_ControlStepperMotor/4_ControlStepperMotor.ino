/**
 * Experiment: Controlling A Stepper Motor With Your Muscles
 * Link: https://backyardbrains.com/experiments/MuscleSpikerShield_StepperMotor
 * 
 * Ported By: Ben Antonellis
 * Date: February 4th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

#define STEPS 48  						// The number of steps the engine has.
Stepper motor(STEPS, 2, 3, 4, 5);  		// Specifies the number of engine steps and pins.

#define MAX 20   						// Maximum reading possible. PLAY WITH THIS VALUE!
#define MAX_STEPS 10 					// This is the maximum number of steps that will advance (You can modify this value).
int readings[10];  						// Array of readings.
int finalReading;						// Averaged out reading from readings array.
byte multiplier = 1;					// Multiplier for analog readings.
byte numSteps = 0;						// The number of steps to take.
int currentSteps = 0;					// The current number of steps taken.

void setup() {

	board.startMeasurements();
	motor.setSpeed(200);				// The RPMs engine speed is specified.

}

void loop() {

	for (int i = 0; i < 10; i++) {
		readings[i] = board.getNewSample() * multiplier;
		finalReading += readings[i];
	}
	
	finalReading /= 10;
	finalReading = constrain(finalReading, 0, MAX);
	numSteps = map(finalReading, 0, MAX, 0, MAX_STEPS);

	if (numSteps == 0) {
		motor.step(0);
	} else {
		for (currentSteps = 0; currentSteps <= numSteps; currentSteps++) {
			Serial.println(currentSteps);
			motor.step(1);
			delay(50);
		}
	}

	delay(10);

}