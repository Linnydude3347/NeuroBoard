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
 * @date January 6th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	// **************************************** //

	board.startMeasurements();
	board.startServo();

	// **************************************** //

	// Alternatively, we could set the servo based on a button pressed //

	board.enableButtonPress(WHITE_BTN, []() {
		board.startServo();
	});

	// **************************************** //

	// We could also start the servo, and set sensitivity calls to the buttons //

	board.startServo();

	board.enableButtonPress(RED_BTN, []() {
		board.increaseSensitivity();
	});

	board.enableButtonPress(WHITE_BTN, []() {
		board.decreaseSensitivity();
	});

	// **************************************** //

}

void loop() {

	// If we want a visual representation of EMB strength, we can call the below function //

	board.displayEMGStrength();

	// loop code here

}