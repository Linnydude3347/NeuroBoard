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

	board.enableButtonPress(RED_BTN, []() {
		Serial.println("Red Button Pressed.");
	});

	board.enableButtonLongPress(WHITE_BTN, 1000, []() {
		Serial.println("White Button Pressed.");
	});

	board.setTriggerOnEnvelope(500, []() {
		Serial.println("Threshold Reached!");
	});
 
}

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
