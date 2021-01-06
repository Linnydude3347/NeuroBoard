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

	int sample = board.getNewSample();
    //int envelopeValue = board.getEnvelopeValue();

	//Serial.println(sample);
    //delay(25);

}
