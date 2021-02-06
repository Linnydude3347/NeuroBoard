/**
 * Simple program that controls the servo with analog readings.
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
 * @date January 7th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	// Required to start receiving samples from the board //
	board.startMeasurements();

	// **************************************** //
	// OPTION 1 //

	// You can start the servo by simplying typing this line in the setup function

	board.startServo();

	// **************************************** //
	// OPTION 2 //

	// Alternatively, we could set the servo based on a button pressed //

	board.enableButtonPress(WHITE_BTN, []() {
		board.startServo();
	});

	// **************************************** //
	// OPTION 3 //

	// We could also start the servo, and set sensitivity calls to the buttons //

	board.startServo();

	board.enableButtonPress(RED_BTN, []() {
		board.increaseSensitivity();
	});

	board.enableButtonPress(WHITE_BTN, []() {
		board.decreaseSensitivity();
	});

	// **************************************** //
	// OPTION 4 //

	// We could start the servo, and set the buttons to change the default servo mode //

	board.startServo();

	board.enableButtonPress(RED_BTN, []() {
		board.setServoDefaultPosition(OPEN_MODE);
	});

	board.enableButtonPress(WHITE_BTN, []() {
		board.setServoDefaultPosition(CLOSED_MODE);
	});

	// **************************************** //

}

void loop() {

	// Required if any button/envelopeTrigger/servo is enabled
	board.handleInputs();

	// If we want a visual representation of EMB strength, we can call the below function //

	board.displayEMGStrength();

	// loop code here

}