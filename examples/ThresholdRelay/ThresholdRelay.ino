/**
 * Simple program that prints when a threshold is reached.
 * 
 * MAC:
 * 	View Console: Shift + Command + M
 * 	View Plotter: Shift + Command + L
 * 
 * WINDOWS:
 * 	View Console: Control + Shift + M
 *  View Plotter: Control + Shift + L
 * 
 * @author Ben Antonellis
 * @date January 7th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	// Required to start receiving samples from the board //
	board.startMeasurements();

	// Once the incoming sample reaches above set threshold (700 in this case), the
	// function passed is called. Then, once the samples reached 9/10th of the passed
	// threshold (630 in this case), the function will be allowed to call again.

	// Once the threshold is met, the relay is turned on. It is only turned off when
	// the incoming samples are below the second threshold.

	board.setTriggerOnEnvelope(700, []() {
		Serial.println("Threshold Reached!");
	});

	// You can also set your own second threshold

	board.setTriggerOnEnvelope(600, 400, []() {
		Serial.println("Threshold Reached!");
	});

}

void loop() {

	// Required if any button/envelopeTrigger/servo is enabled
	board.handleInputs();

	// loop code here

}