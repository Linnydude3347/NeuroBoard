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
 * @date January 5th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	board.startMeasurements();

}

unsigned long oneSecond = 0;
unsigned long twoSeconds = 0;
unsigned long threeSeconds = 0;

void loop() {

	if (board.wait(1000, oneSecond)) {
		Serial.println("One Second Passed.");
	}
	if (board.wait(2000, twoSeconds)) {
		Serial.println("Two Seconds Passed.");
	}
	if (board.wait(3000, threeSeconds)) {
		Serial.println("Three Seconds Passed.");
	}

}