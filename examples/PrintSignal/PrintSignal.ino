/**
 * Simple program that prints the signal recieved from the
 * spike recorder to the Arduino Serial Console/Plotter
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

void loop() {

	int sample = board.getNewSample();
	int envelopeValue = board.getEnvelopeValue();

	Serial.println(sample);

}