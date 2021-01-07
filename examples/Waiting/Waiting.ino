/**
 * Simple program that demonstrates how to use the wait function.
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

}

// Define variables to hold the count of the wait functions

unsigned long oneSecond = 0;
unsigned long twoSeconds = 0;
unsigned long threeSeconds = 0;

void loop() {

	// Prints every one second (1000 milliseconds)
	if (board.wait(1000, oneSecond)) {
		Serial.println("One Second Passed.");
	}
	// Prints every two seconds (2000 milliseconds)
	if (board.wait(2000, twoSeconds)) {
		Serial.println("Two Seconds Passed.");
	}
	// Prints every three seconds (3000 milliseconds)
	if (board.wait(3000, threeSeconds)) {
		Serial.println("Three Seconds Passed.");
	}

}