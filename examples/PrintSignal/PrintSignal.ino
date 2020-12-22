/**
 * Simple program that prints the signal recieved from the
 * spike recorder to the Arduino Serial Console/Plotter
 * 
 * View Console: Shift + Command + M
 * View Plotter: Shift + Command + L
 * 
 * @author Ben Antonellis
 * @date 09-24-20
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	board.startMeasurements();

}

void loop() {

	int sample = board.getNewSample();
	int envelopeValue = board.getEnvelopeValue();

	Serial.println(sample);

}