/**
 * NeuroBoard Test Implementation File.
 * 
 * @author Ben Antonellis
 * @date October 15th, 2020
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

unsigned long oneSecond = 0;
unsigned long twoSeconds = 0;
unsigned long threeSeconds = 0;

void setup() {

	board.startMeasurements();
	board.setChannel(A0);
	board.setDecayRate(10);

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

	// LED CODE
	//for (int i = 0; i < 8; i++) {
	//	board.writeLED(i, ON);
	//	delay(100);
	//}
	//for (int i = 0; i < 8; i++) {
	//	board.writeLED(i, OFF);
	//	delay(100);
	//}
	// LED CODE

	int sample = board.getNewSample();
	int ev = board.getEnvelopeValue();
	//Serial.println(sample); // Remove LED code if you want realistic sampling time
	Serial.println(ev);
	delay(10);

}
