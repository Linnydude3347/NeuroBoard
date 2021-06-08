/**
    NeuroBoard.cpp - A library for interacting with the Neuroduino Board.
    Copyright (C) 2021 Backyard Brains.

    This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

    This library is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
    Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public
    License along with this library; if not, write to the Free Software
    Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301
    USA.
    
    Backyard Brains hereby disclaims all copyright interest in the library
    `NeuroBoard` (a library for interacting with the Neuroduino Board) written
    by Benjamin Antonellis.

    Backyard Brains, March 2021.
**/

/**
 * NeuroBoard Implementation File.
 *
 * @author Ben Antonellis
 * @date January 7th, 2021
**/

#include "NeuroBoard.hpp"

/* ******************************************************* */
/** @author Stanislav Mircic **/

#define SHIFT_LATCH_PIN    B00000100                        // latch pin for shift register RCK - PB2
#define I_SHIFT_LATCH_PIN  B11111011
#define SHIFT_CLOCK_PIN    B00000010                        // clock pin for shift register PB1
#define I_SHIFT_CLOCK_PIN  B11111101
#define SHIFT_DATA_PIN     B00001000                        // serial data pin for shift register SER - PB3
#define I_SHIFT_DATA_PIN   B11110111
#define BITMASK_ONE        B00000001
#define I_BITMASK_ONE      B11111110

/* ******************************************************* */

/// PRIVATE FUNCTIONS ///

void sendMessage(const char*);
long fasterMap(long, long, long, long, long);

// Button Wait Variables //

ulong redCount = 0;
ulong whiteCount = 0;
ulong redDebounceCount = 0;
ulong whiteDebounceCount = 0;

// Button Variables //

Button redButtonTrigger = Button();
Button whiteButtonTrigger = Button();
Button redLongButtonTrigger = Button();
Button whiteLongButtonTrigger = Button();

// Trigger Variable //

Trigger envelopeTrigger = Trigger();

// Servo Variable //

NeuroServo servo = NeuroServo();
bool servoEnabled = false;
bool emgStrengthEnabled = false;

uint8_t NeuroBoard::decayRate = 1;

// Buffer Variables //

int* buffer = new int[BUFFER_SIZE];
int head = 0;
int tail = 0;
bool full = false;

// Serial Buffer Variables //

#define COMMAND_BUFFER_SIZE 30
#define OUTPUT_FRAME_BUFFER_SIZE 64
#define ANTI_FLICKERING_TIME_IN_MS 50

uint16_t antiFlickeringCounterMax;
uint16_t antiFlickeringTimerForOutput;
uint8_t movingThreshold;

char commandBuffer[COMMAND_BUFFER_SIZE];
int commandBufferIndex = 0;
bool messageReceived = false;

byte outputFrameBuffer[OUTPUT_FRAME_BUFFER_SIZE];
bool NeuroBoard::communicationEnabled = false;
bool outputFrameReady = false;
int numberChannels = 1;

byte escapeSequence[ESCAPE_SEQUENCE_LENGTH] = { 255, 255, 1, 1, 128, 255 };
byte endOfescapeSequence[ESCAPE_SEQUENCE_LENGTH] = { 255, 255, 1, 1, 129, 255 };

// Envelope Value //

int envelopeValue;

uint8_t NeuroBoard::channel = A0;

// Variables for button holding //

int RBD = 0;                // Red Button Down
ulong RBT = 0;              // Red Button Time
int RBC = 0;                // Red Button Collected

int WBD = 0;                // White Button Down
ulong WBT = 0;              // White Button Time
int WBC = 0;                // White Button Collected

int redLongButtonHeld = 0;
int whiteLongButtonHeld = 0;

int redLongCalled = 0;
int whiteLongCalled = 0;

// Sample variable //

int reading;

// ISR //
#ifdef ARDUINO_AVR_LEONARDO
ISR (TIMER3_COMPA_vect) {
#else // Arduino Uno Board
ISR (TIMER0_COMPA_vect) {
#endif

    // Get reading from analog //

    reading = analogRead(NeuroBoard::channel);

    // Calculate envelope value here //

    envelopeValue = (reading >= envelopeValue) ? (reading) : (envelopeValue - NeuroBoard::decayRate);

    // Place new reading in buffer //

    buffer[head] = reading;

    if (full) {
        tail = (tail == BUFFER_SIZE) ? (0) : (tail + 1);
    }
    head = (head == BUFFER_SIZE) ? (0) : (head + 1);
    full = head == tail;

	// Populate output buffer with sample data //

	if (NeuroBoard::communicationEnabled) {

		// Because the buffer is six (6) elements, we need to populate the output frame buffer with the buffer data.

		outputFrameBuffer[0] = (buffer[0] >> 7) | 0x80;
		outputFrameBuffer[1] = buffer[0] & 0x7F;
		outputFrameBuffer[2] = (buffer[1] >> 7) & 0x7F;
		outputFrameBuffer[3] = buffer[1] & 0x7F;
		outputFrameBuffer[4] = (buffer[2] >> 7) & 0x7F;
		outputFrameBuffer[5] = buffer[2] & 0x7F;
		outputFrameBuffer[6] = (buffer[3] >> 7) & 0x7F;
		outputFrameBuffer[7] = buffer[3] & 0x7F;
		outputFrameBuffer[8] = (buffer[4] >> 7) & 0x7F;
		outputFrameBuffer[9] = buffer[4] & 0x7F;
		outputFrameBuffer[10] = (buffer[5] >> 7) & 0x7F;
		outputFrameBuffer[11] = buffer[5] & 0x7F;

		// Set ready flag so we know when to write data.

		outputFrameReady = true;

	}

}

// PUBLIC METHODS //

void NeuroBoard::startMeasurements(void) const {

    // Start Serial //

    Serial.begin(SERIAL_CAP);

    // Set pin modes for Stanislav's code //

    pinMode(14, OUTPUT); // MISO
    pinMode(15, OUTPUT); // SCK
    pinMode(16, OUTPUT); // MOSI

    // Set relay pin //

    pinMode(RELAY_PIN, OUTPUT);

    // Initialize timer //

    // Disable interrupts //

    noInterrupts();

	#ifdef ARDUINO_AVR_LEONARDO
	for (int i = 0; i <= 7; i++) // [0, 7]
		pinMode(i, OUTPUT);
	#endif

	sbi(ADCSRA, ADPS2);
	cbi(ADCSRA, ADPS1);
	cbi(ADCSRA, ADPS0);

	// Set timer register flags //

	TCCR3A = 0;
	TCCR3B = 0;
	TCNT3 = 0;

	// Configure timer registers //

	OCR3A = 31250;
	TCCR3B = (TCCR3B & 0xF8) | 0x01;
	TIMSK3 |= (1 << OCIE1A);

	// Enable interrupts //

    interrupts();

}

void NeuroBoard::startCommunication(void) {

	NeuroBoard::communicationEnabled = true;

	antiFlickeringCounterMax = ((ANTI_FLICKERING_TIME_IN_MS * 10) / numberChannels);
	antiFlickeringTimerForOutput = antiFlickeringCounterMax;

}

#ifdef ARDUINO_AVR_LEONARDO
	bool redPressed(void) { return PIND & B00010000; }
	bool whitePressed(void) { return PINE & B01000000; }
#else // Arduino Uno Board
	// Placeholders until we get correct ports and pins
	bool redPressed(void) { /*return PIND & B00001000;*/ return digitalRead(4); }
	bool whitePressed(void) { /*return PINE & B10000000;*/ return digitalRead(7); }
#endif

void NeuroBoard::handleInputs(void) {

    // Check if buttons are enabled //

    if (redButtonTrigger.enabled) {
        if (redPressed()) {
            RBD = 1;
            if (!RBC) {
                RBT = millis();
                RBC = 1;
            }
        } else {
            if (RBD) {
                if ((millis() - RBT) <= 250) {
                    redButtonTrigger.callback();
                }
                RBD = 0;
                RBC = 0;
            }
        }
    }

    if (whiteButtonTrigger.enabled) {
        if (whitePressed()) {
            WBD = 1;
            if (!WBC) {
                WBT = millis();
                WBC = 1;
            }
        } else {
            if (WBD) {
                if ((millis() - WBT) <= 250) {
                    whiteButtonTrigger.callback();
                }
                WBD = 0;
                WBC = 0;
            }
        }
    }

    if (redLongButtonTrigger.enabled) {
        if (redPressed()) {
            if (redLongButtonHeld) {
                if (wait(redLongButtonTrigger.interval, redCount)) {
                    if (!redLongCalled) {
                        redLongButtonTrigger.callback();
                        redLongButtonHeld = 0;
                        redLongCalled = 1;
                    }
                }
            }
            redLongButtonHeld = 1;
        } else {
            redLongButtonHeld = 0;
            redLongCalled = 0;
            redCount = millis();
        }
    }

    if (whiteLongButtonTrigger.enabled) {
        if (whitePressed()) {
            if (whiteLongButtonHeld) {
                if (wait(whiteLongButtonTrigger.interval, whiteCount)) {
                    if (!whiteLongCalled) {
                        whiteLongButtonTrigger.callback();
                        whiteLongButtonHeld = 0;
                        whiteLongCalled = 1;
                    }
                }
            }
            whiteLongButtonHeld = 1;
        } else {
            whiteLongButtonHeld = 0;
            whiteLongCalled = 0;
            whiteCount = millis();
        }
    }

    // Check if envelope trigger is set //

    if (envelopeTrigger.enabled) {

        if (envelopeValue >= envelopeTrigger.threshold) {
            if (!envelopeTrigger.thresholdMet) {
                envelopeTrigger.thresholdMet = true;
                envelopeTrigger.callback();
            }
        } else {
            if (envelopeValue <= envelopeTrigger.secondThreshold) {
                envelopeTrigger.thresholdMet = false;
            }
        }

    }

    // Servo Code //

    if (servoEnabled) {

        servo.analogReadings = reading;

        // Set new angle if enough time passed
        if (millis() - servo.oldTime > MINIMUM_SERVO_UPDATE_TIME) {

            // Calculate new angle for servo
            if (servo.currentFunctionality == OPEN_MODE) {
                servo.analogReadings = constrain(servo.analogReadings, 40, servo.emgSaturationValue);
                servo.newDegree = fasterMap(servo.analogReadings, 40, servo.emgSaturationValue, 190, 105);
            } else {
                servo.analogReadings = constrain(servo.analogReadings, 120, servo.emgSaturationValue);
                servo.newDegree = fasterMap(servo.analogReadings, 120, servo.emgSaturationValue, 105, 190);
            }

            // Check if we are in servo dead zone
            if (abs(servo.newDegree - servo.oldDegrees) > GRIPPER_MINIMUM_STEP) {
                // Set new servo angle
                servo.Gripper.write(servo.newDegree);
            }

            // Set old time and degrees for new calculation
            servo.oldTime = millis();
            servo.oldDegrees = servo.newDegree;

        }

    }

    // EMG Strength Code //

	int readings = constrain(reading, 30, servo.emgSaturationValue);
	movingThreshold = fasterMap(readings, 30, servo.emgSaturationValue, 0, MAX_LEDS);
	movingThreshold = servo.sensitivities[movingThreshold]; // Convert index to value.

    if (emgStrengthEnabled) {

        // Turn OFF all LEDs on LED bar
        for (int i = 0; i < MAX_LEDS; i++) {
            this->writeLED(this->ledPins[i], OFF);
        }

        // Calculate what LEDs should be turned ON on the LED bar
        servo.ledbarHeight = fasterMap(readings, 30, servo.emgSaturationValue, 0, MAX_LEDS);

        // Display fix for when servo is disabled, but user still wants visual feedback
        // Check is for a Leonardo and Uno Board. (NEED MORE TESTING BEFORE PUSH!).

        if (!servoEnabled and servo.ledbarHeight == 7 and MAX_LEDS == 8) {
            servo.ledbarHeight++;
        }

		if (!servoEnabled and servo.ledbarHeight == 5 and MAX_LEDS == 6) {
			servo.ledbarHeight++;
		}

        // Turn ON LEDs on the LED bar
        for (int i = 0; i < servo.ledbarHeight; i++) {
            this->writeLED(this->ledPins[i], ON);
        }

    }

	if (!NeuroBoard::communicationEnabled) return; // We can return from here because nothing else will be after the serial buffer stuff.

	//// Serial Stuff Here ////

	// Relay Threshold //

	antiFlickeringTimerForOutput--;
	if (antiFlickeringTimerForOutput == 0) {
		antiFlickeringTimerForOutput = antiFlickeringCounterMax;
		if (envelopeValue > movingThreshold) {
			PORTD |= BITMASK_ONE; // Turn ON relay.
		} else {
			PORTD &= I_BITMASK_ONE; // Turn OFF relay.
		}
	}

	// Serial Communication //

	if (outputFrameReady) {

		Serial.write(outputFrameBuffer, (numberChannels << 1));
		outputFrameReady = false;

	}

	while (Serial.available() > 0) {

		byte incoming = Serial.read();

		if (incoming == 10) { // Newline character, we've reached end of message.
			commandBuffer[commandBufferIndex] = 0;
			messageReceived = true;
			commandBufferIndex = 0;
		} else {
			commandBuffer[commandBufferIndex] = incoming;
			commandBufferIndex++;
			if (commandBufferIndex == COMMAND_BUFFER_SIZE) { // If incoming data is longer than the buffer size, start writing from beginning.
				commandBufferIndex = 0;
				break;
			}
		}

	}

	if (messageReceived) {

		messageReceived = false;
		commandBufferIndex = 0;

		char* command = strtok(commandBuffer, ";");
		while (command != 0) {

			char* separator = strchr(command, ':');
			if (separator != 0) {
				*separator = 0;
				--separator;
				if (*separator == 'c') { // Request for channels.
					separator += 2;
					TIMSK3 &= ~(1 << OCIE1A); // Disable timer for sampling.
					PORTD &= I_BITMASK_ONE; // Turn off relay.
					antiFlickeringCounterMax = ((ANTI_FLICKERING_TIME_IN_MS * 10) / numberChannels);
					antiFlickeringTimerForOutput = antiFlickeringCounterMax;
					TIMSK3 |= (1 << OCIE1A); // Enable timer for sampling.
				}
				if (*separator == 's') { // Request for sampling rate.
					// Do nothing.
				}
				if (*separator == 'b') { // Request for impuls.
					TIMSK3 &= ~(1 << OCIE1A); // Disable timer for sampling.
					PORTD &= I_BITMASK_ONE; // Turn off relay.
					sendMessage(CURRENT_SHIELD_TYPE);
					TIMSK3 |= (1 << OCIE1A); // Enable timer for sampling.
				}
			}

			command = strtok(0, ";"); // Find next command.

		}

	}

}

void NeuroBoard::startServo(void) const {

    // Ensure servo isn't already enabled before starting
    if (servoEnabled) return;

    // Attach servo to board
    servo.Gripper.attach(SERVO_PIN);

    PORTD &= I_BITMASK_ONE; // digitalWrite(RELAY_PIN, OFF);

    // Initialize all LED pins to output
    for (int i = 0; i < MAX_LEDS; i++) {
        pinMode(this->ledPins[i], OUTPUT);
    }

    // Get current sensitivity
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];

    // Set servo enabled boolean
    servoEnabled = true;

}

void NeuroBoard::endServo(void) const {

    // Ensure servo is enabled before disabling
    if (!servoEnabled) return;

    // Set servo boolean value to false
    servoEnabled = false;

    // Detach servo
    servo.Gripper.detach();

    // Reset servo object to default values
    servo = NeuroServo();

}

void NeuroBoard::increaseSensitivity(void) const {

    // Ensure servo is enabled before modifying sensitivity value
    if (!servoEnabled || servo.lastSensitivitiesIndex == 5) return; // 5 equals end of sensitivity array

    // Increment sensitivity index
    servo.lastSensitivitiesIndex++;

    // Get current sensitivity value
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];

}

void NeuroBoard::decreaseSensitivity(void) const {

    // Ensure servo is enabled before modifying sensitivity value
    if (!servoEnabled || servo.lastSensitivitiesIndex == 0) return;

    // Decrement sensitivity index
    servo.lastSensitivitiesIndex--;

    // Get current sensitivity value
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];

}

void NeuroBoard::setServoDefaultPosition(const int& position) const {

    servo.currentFunctionality = position;

}

int NeuroBoard::getNewSample(void) const {

    int value = buffer[tail]; // Can't just return this because tail is changed below //
    full = false;
    tail = (tail == BUFFER_SIZE) ? (0) : (tail + 1);

    return value;

}

void NeuroBoard::getSamples(int* arr[], const int& size) const {

    *arr = new int[size];
    for (int i = 0; i < size; i++) {
        // We can't just access the buffer directly because we need to modify it once retrieving a value.
        (*arr)[i] = this->getNewSample();
    }

}

int NeuroBoard::getEnvelopeValue(void) const {

    return envelopeValue;

}

void NeuroBoard::setChannel(const uint8_t& newChannel) {

    NeuroBoard::channel = newChannel;

}

void NeuroBoard::setDecayRate(const uint8_t& rate) {

    NeuroBoard::decayRate = rate;

}

void NeuroBoard::enableButtonPress(const uint8_t& button, void (*callback)(void)) const {

	if (button == RED_BTN)
		redButtonTrigger.set(callback);
	if (button == WHITE_BTN)
		whiteButtonTrigger.set(callback);

}

void NeuroBoard::enableButtonLongPress(const uint8_t& button, const int& milliseconds, void (*callback)(void)) const {

	if (button == RED_BTN)
		redLongButtonTrigger.set(callback, milliseconds);
	if (button == WHITE_BTN)
		whiteLongButtonTrigger.set(callback, milliseconds);

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, const int& secondFactor, void (*callback)(void)) const {

    envelopeTrigger.set(threshold, secondFactor, callback);

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, void (*callback)(void)) const {

    this->setTriggerOnEnvelope(threshold, threshold - (threshold / 10), callback);

}

void NeuroBoard::displayEMGStrength(void) const {

    emgStrengthEnabled = !emgStrengthEnabled;

}

bool wait(const int& milliseconds, ulong& variable) {

    ulong ms = millis();
    bool done = (ms - variable) >= milliseconds;
    if (done)
		variable = ms;
    return done;

}

/* ******************************************************* */
/** @author Stanislav Mircic **/
/** Simplified by Ben Antonellis **/

void NeuroBoard::writeLEDs(void) const {

    PORTB &= I_SHIFT_LATCH_PIN;
    byte tempBitmask = BITMASK_ONE;

    for (int i = 0; i < MAX_LEDS; i++) {

        PORTB = (_shiftRegState & tempBitmask) ? (PORTB | SHIFT_DATA_PIN) : (PORTB & I_SHIFT_DATA_PIN);

        //pulse the clock for shift
        PORTB |= SHIFT_CLOCK_PIN;
        PORTB &= I_SHIFT_CLOCK_PIN;
        tempBitmask <<= 1;

    }

    PORTB |= SHIFT_LATCH_PIN;
}

void NeuroBoard::writeLEDs(byte outByte) {

    PORTB &= I_SHIFT_LATCH_PIN;
    _shiftRegState = outByte;
    byte tempBitmask = BITMASK_ONE;

    for (int i = 0; i < MAX_LEDS; i++) {

        PORTB = (outByte & tempBitmask) ? (PORTB | SHIFT_DATA_PIN) : (PORTB & I_SHIFT_DATA_PIN);

        //pulse the clock for shift
        PORTB |= SHIFT_CLOCK_PIN;
        PORTB &= I_SHIFT_CLOCK_PIN;
        tempBitmask <<= 1;

    }

    PORTB |= SHIFT_LATCH_PIN;
}

void NeuroBoard::writeLED(const int& led, const bool& state) {

    const int LED = this->ledPins[led];

    byte bitMask = BITMASK_ONE;

    #ifdef ARDUINO_AVR_UNO
        if (!(8 < LED < 13)) return;
    #else // We must be dealing with a Leonardo
        if (!(0 < LED < 7)) return;
    #endif

    if (state) {
        _shiftRegState |= bitMask << (7 - LED);
    } else {
        _shiftRegState &= ~(bitMask << (7 - LED));
    }

    writeLEDs();

}

/**
 * Pushes a message to the main sending buffer.
 * 
 * @param message Char array representing type of board.
 * 
 * @return void.
**/
void sendMessage(const char* message) {

	int i;
	int head = 0;

	for (i = 0; i < ESCAPE_SEQUENCE_LENGTH; i++)
		outputFrameBuffer[head++] = escapeSequence[i];
	
	i = 0;
	while (message[i] != 0)
		outputFrameBuffer[head++] = message[i++];
	
	for (i = 0; i < ESCAPE_SEQUENCE_LENGTH; i++)
		outputFrameBuffer[head++] = endOfescapeSequence[i];

	Serial.write(outputFrameBuffer, head);

}

/**
 * Faster version of map() that doesn't use multiplication or division.
 * 
 * @param value Value to change
 * @param fromLow Original low value.
 * @param fromHigh Original high value.
 * @param toLow New low value.
 * @param toHigh New high value.
 * 
 * @return long - Newly mapped value.
**/
long fasterMap(long value, long fromLow, long fromHigh, long toLow, long toHigh) {

    long first = value - fromLow;
    long second = toHigh - toLow;
    long combined = 0;
    while (second > 0) {                   // Avoid usage of * operator
        combined = combined + first;
        second = second - 1;
    }
    long third = fromHigh - fromLow;
    long count = 0;
    while (combined >= third) {            // Avoid usage of / operator
        combined = combined - third;
        count = count + 1;
    }
    count = count + toLow;
    return count;

}

/* ******************************************************* */
