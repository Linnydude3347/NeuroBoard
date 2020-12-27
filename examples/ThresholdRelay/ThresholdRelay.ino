/**
 * Simple program that prints when a threshold is reached.
 * 
 * MAC:
 * 	View Console: Shift + Command + M
 * 	View Plotter: Shift + Command + L
 * 
 * WINDOWS:
 * 	View Console: Control + Shift + M
 *  View Plotter: Control + Shift + L
 * 
 * @author Ben Antonellis
 * @date September 24th, 2020
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