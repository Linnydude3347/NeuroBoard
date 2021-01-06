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
	board.startServo();

	board.enableButtonPress(RED_BTN, []() {
		Serial.println("Red Button Pressed.");
	});

	board.enableButtonLongPress(RED_BTN, 1000, []() {
		Serial.println("Red Button Held.");
	});

	board.enableButtonPress(WHITE_BTN, []() {
		Serial.println("White Button Pressed.");
	});

	board.enableButtonLongPress(WHITE_BTN, 1000, []() {
		Serial.println("White Button Held.");
	});

	board.setTriggerOnEnvelope(500, []() {
		Serial.println("Threshold Reached!");
	});

}

void loop() {

	int sample = board.getNewSample();
	int ev = board.getEnvelopeValue();
	Serial.println(sample); // Remove LED code if you want realistic sampling time

}
