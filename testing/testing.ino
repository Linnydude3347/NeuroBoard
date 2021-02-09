/**
 * Pseudo unittest file. DO NOT USE.
 * 
 * @author Ben Antonellis
 * @date February 8th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

bool testDecayRate() {

	int newDecayRate = 10;
	board.setDecayRate(newDecayRate);
	return NeuroBoard::decayRate == newDecayRate;

}

bool testChannel() {

	uint8_t newChannel = A2;
	board.setChannel(newChannel);
	return NeuroBoard::channel == newChannel;

}

void setup() { board.startMeasurements(); }

void loop() {

	int passed = 0;
	int tests = 2;

	if (testDecayRate()) passed++;
	if (testChannel()) passed++;

	double percent = passed / tests;

	Serial.print("Tests passed: ");
	Serial.print(passed);
	Serial.print("/");
	Serial.print(tests);
	Serial.print(" (");
	Serial.print(percent);
	Serial.println("%)");

	return;

}

