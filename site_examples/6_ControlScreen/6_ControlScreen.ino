/**
 * Experiment: Controlling An LCD Screen With Your Muscles.
 * Link: https://backyardbrains.com/experiments/MuscleSpikerShield_LCD
 * 
 * Ported By: Ben Antonellis
 * Date: February 8th, 2021
**/

#include "NeuroBoard.hpp"

NeuroBoard board;

#include <LiquidCrystal.h>
LiquidCrystal screen(7, 6, 5, 4, 3, 2);

#define MAX 8
#define NUMBER_OF_COLUMNS 16
int finalReading;
byte currentLCD = 0;
byte multiplicator = 1;
byte columnX = 0;

byte a1[8] = { B10000, B01000, B00100, B00100, B00111, B01000, B10000, B11111 };
byte a2[8] = { B00000, B01000, B01001, B01001, B11111, B00000, B00000, B11111 };
byte a3[8] = { B01000, B10000, B00000, B00000, B11000, B10110, B01001, B11001 };
byte a4[8] = { B00000, B00000, B00000, B00000, B00010, B00100, B11000, B00000 };
byte a5[8] = { B11111, B10000, B01000, B00111, B00000, B00000, B00000, B00000 };
byte a6[8] = { B11111, B00000, B00000, B11111, B01001, B01001, B01000, B00000 };
byte a7[8] = { B11001, B01001, B10110, B11000, B00000, B00000, B10000, B01000 };
byte a8[8] = { B00000, B11000, B00100, B00010, B00000, B00000, B00000, B00000 };

void setup() {

	board.startMeasurements();

	screen.begin(16, 2);								// (columns, rows) of LCD screen.

	screen.createChar(1, a1);
	screen.createChar(2, a2);
	screen.createChar(3, a3);
	screen.createChar(4, a4);
	screen.createChar(5, a5);
	screen.createChar(6, a6);
	screen.createChar(7, a7);
	screen.createChar(8, a8);

	screen.setCursor(1, 0);
	for (int i = 1; i <= 4; i++) {
		screen.write(i);
	}
	screen.setCursor(1, 1);
	for (int i = 5; i <= 8; i++) {
		screen.write(i);
	}

	screen.setCursor(7, 0);
	screen.print("BACKYARD");
	screen.setCursor(8, 1);
	screen.print("BRAINS");

	delay(2000);
	screen.clear();

}

void loop() {

	int* reading;
	board.getSamples(&reading, 10);
	for (int i = 0; i < 10; i++) {
		finalReading += reading[i] * multiplicator;
	}
	finalReading /= 10;
	delete[] reading;

	finalReading = constrain(finalReading, 0, MAX);
	currentLCD = map(finalReading, 0, MAX, 0, NUMBER_OF_COLUMNS);

	for (columnX = 0; columnX < currentLCD; columnX++) {
		screen.setCursor(columnX, 0);
		screen.write(255);
		screen.setCursor(columnX, 1);
		screen.write(255);
		delay(30);
	}

	for (columnX = currentLCD; columnX < 16; columnX++) {
		screen.setCursor(columnX, 0);
		screen.write(254);
		screen.setCursor(columnX, 1);
		screen.write(254);
	}
	delay(10);

}