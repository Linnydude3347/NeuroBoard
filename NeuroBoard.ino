/**
 * NeuroBoard Test Implementation File.
 * All code in this file is subject to change.
 * 
 * @author Ben Antonellis
 * @date October 15th, 2020
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	board.startMeasurements();
	board.setChannel(A0);
	board.setDecayRate(10);

	board.enableButtonPress(WHITE_BTN, []() {
    	Serial.println("white press");
	});

	board.enableButtonPress(RED_BTN, []() {
    	Serial.println("red press");
	});

	board.enableButtonLongPress(WHITE_BTN, 1000, []() {
    	Serial.println("white long press");
	});

	board.enableButtonLongPress(RED_BTN, 1000, []() {
    	Serial.println("red long press");
	});

}

void loop() {

	int sample = board.getNewSample();
	int ev = board.getEnvelopeValue();
	//Serial.println(sample); // Remove LED code if you want realistic sampling time

}
