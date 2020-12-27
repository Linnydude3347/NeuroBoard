/**
 * Program to demonstrate the uses and limitations of the setChannel function.
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

	// There are a couple ways to set the channel to listen on. This is because
	// not everyone may be familiar with how channels are named.

	// To those unfamiliar with the Arduino and their analog naming
	// system, you can set the channel like so:

	board.setChannel(0); // Listens on A0, which is the 1st analog.
	board.setChannel(11); // Listens on A11, which is the 12th analog.

	// Any number N, where 0 <= N <= 11, will work.

	// To those who are familiar with Arduino's analog names, you can pass them
	// directly in to the function.

	board.setChannel(A0); // Listens on A0, which is the 1st analog.
	board.setChannel(A11); // Listens on A11, which is the 12th analog.

	// Any AN, where 0 <= N <= 11, will work.

}

void loop() {

	// loop code here

}