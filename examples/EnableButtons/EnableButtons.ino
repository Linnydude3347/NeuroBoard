/**
 * Simple program to demonstrate how to set buttons properly.
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

	// Set a trigger for a regular various button presses (RED or WHITE)
	
	board.enableButtonPress(WHITE_BTN, []() {
		Serial.println("White Button Pressed!");
	});
	
	board.enableButtonPress(RED_BTN, []() {
		Serial.println("Red Button Pressed!");
	});
	
	board.enableButtonLongPress(WHITE_BTN, 1000, []() {
    	Serial.println("White Button Held!");
	});

	board.enableButtonLongPress(RED_BTN, 1000, []() {
    	Serial.println("Red Button Held!");
	});
	
}

void loop() {

	// Required if any button/envelopeTrigger/servo is enabled
	board.handleInputs();

	// loop code here

}