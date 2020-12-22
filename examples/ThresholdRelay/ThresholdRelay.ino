/**
 * Simple program that prints when a threshold is reached.
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

	auto printLambda = []() { Serial.println("Threshold Reached!"); };

	board.setTriggerOnEnvelope(700, printLambda);

}