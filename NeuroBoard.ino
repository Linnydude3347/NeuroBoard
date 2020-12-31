/**
 * NeuroBoard Test Implementation File.
 * 
 * @author Ben Antonellis
 * @date October 15th, 2020
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	board.startMeasurements();
	board.setChannel(A0);

}

void loop() {

	int sample = board.getNewSample();
    int envelopeValue = board.getEnvelopeValue();

	board.enableButtonPress(WHITE_BTN, [](){ Serial.println("White Button Pressed!"); }, 250);

	Serial.println(sample);
    delay(25);

}