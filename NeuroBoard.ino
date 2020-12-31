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

	board.enableButtonPress(RED_BTN, []() {
		Serial.println("Red Button Pressed");
	});

	board.enableButtonPress(WHITE_BTN, []() {
		Serial.println("White Button Pressed");
	});

}

void loop() {

	//int sample = board.getNewSample();
    //int envelopeValue = board.getEnvelopeValue();

	//Serial.println(sample);
    //delay(25);

}