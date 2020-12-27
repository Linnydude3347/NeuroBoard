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
 * @date December 26th, 2020
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

void setup() {

	board.startMeasurements();

	// Set a trigger for a regular button press (RED_BTN or WHITE_BTN)

	board.enableButtonPress(RED_BTN, [](){ Serial.println("Red Button Pressed!"); });

	// Set a trigger for a long button press (RED_BTN or WHITE_BTN)
	// As an example, the button must be pressed for 1000ms

	board.enableButtonLongPress(WHITE_BTN, 1000, [](){ Serial.println("White Button Held!"); });

	// It should be noted that you cannot set a regular press and a long press
	// for the same button. See below:

	// **************************************************************************************** //

	// NOT ALLOWED

	// board.enableButtonPress(RED_BTN, [](){ Serial.println("Red Button Pressed!"); });
	// board.enableButtonLongPress(RED_BTN, 1000, [](){ Serial.println("Red Button Held!"); });

	// **************************************************************************************** //
	
}

void loop() {

	// loop code here

}