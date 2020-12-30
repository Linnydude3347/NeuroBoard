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
	board.startCommunicaton();

}

void loop() {

	int sample = board.getNewSample();
    int envelopeValue = board.getEnvelopeValue();

	board.enableButtonPress(WHITE_BTN, [](){ Serial.println("Reached!"); }, 250);

	if (board.wait(1000)) {
		Serial.println("1 Second Passed!");
	}

	Serial.println(sample);
    delay(25);

}