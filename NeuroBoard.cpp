/**
 * NeuroBoard Implementation File.
 * 
 * @author Ben Antonellis
 * @date July 5th, 2020
**/

#include "NeuroBoard.hpp"

/* ******************************************************* */
/** @author Stanislav Mircic **/

#define SHIFT_LATCH_PIN B00000100                        //latch pin for shift register        RCK - PB2
#define I_SHIFT_LATCH_PIN B11111011 
#define SHIFT_CLOCK_PIN B00000010                        //clock pin for shift register              PB1
#define I_SHIFT_CLOCK_PIN B11111101 
#define SHIFT_DATA_PIN  B00001000                        //serial data pin for shift register SER - PB3
#define I_SHIFT_DATA_PIN  B11110111 
#define BITMASK_ONE B00000001

/* ******************************************************* */

// Button Variables //

struct button {

    uint8_t _button;
    void (*callback)(void);
    unsigned int interval;

};

typedef struct button Button;

Button redButtonTrigger;
Button whiteButtonTrigger;
Button redLongButtonTrigger;
Button whiteLongButtonTrigger;

bool redButtonSet = false;
bool whiteButtonSet = false;
bool redLongButtonSet = false;
bool whiteLongButtonSet = false;

unsigned int redButtonHoldCount = 0;
unsigned int whiteButtonHoldCount = 0;

// Buffer Variables //

int* buffer = new int[BUFFER_SIZE];
int head = 0;
int tail = 0;
bool full = false;

// Envelope Value //

int envelopeValue;

// Serial variable, changed in startCommunication method //

bool communicate = false;
uint8_t channel = A0;

// ISR //

ISR (TIMER1_COMPA_vect) {

    // Get reading from analog //

    int reading = analogRead(channel);

    // Calculate envelope value here //

    envelopeValue = (reading >= envelopeValue) ? reading : envelopeValue - 1;

    // Place new reading in buffer //

    buffer[head] = reading;

    if (full) {
        tail = (tail + 1) % BUFFER_SIZE;
    }
    head = (head + 1) % BUFFER_SIZE;
    full = head == tail;

    // Check if communication is turned on //

    if (communicate) {

        const uint8_t outputFrameBuffer[2] = { (reading >> 7) | 0x80, reading & 0x7F };

        Serial.write(outputFrameBuffer, 2);

    }

    // Check if buttons are enabled //

    if (redButtonSet) {

        if (digitalRead(redButtonTrigger._button)) {
            if (debounceWait(redButtonTrigger.interval)) {
                redButtonTrigger.callback();
            }
        }

    }

    if (whiteButtonSet) {

        if (digitalRead(whiteButtonTrigger._button)) {
            if (debounceWait(whiteButtonTrigger.interval)) {
                whiteButtonTrigger.callback();
            }
        }

    }

    if (redLongButtonSet) {

        redButtonHoldCount = digitalRead(redLongButtonTrigger._button) ? (redButtonHoldCount + 1) : 0;
        if (redButtonHoldCount == redLongButtonTrigger.interval) {
            redButtonHoldCount = 0;
            redLongButtonTrigger.callback();
        }

    }

    if (whiteLongButtonSet) {

        whiteButtonHoldCount = digitalRead(whiteLongButtonTrigger._button) ? (whiteButtonHoldCount + 1) : 0;
        if (whiteButtonHoldCount == whiteLongButtonTrigger.interval) {
            whiteButtonHoldCount = 0;
            whiteLongButtonTrigger.callback();
        }

    }

}

// PUBLIC METHODS //

void NeuroBoard::startMeasurements(void) {

    // TODO: See documentation.

    // Set pin modes for Stanislav's code //

    pinMode(14, OUTPUT); // MISO
    pinMode(15, OUTPUT); // SCK
    pinMode(16, OUTPUT); // MOSI

    // Initialize timer //

    // Disable interrupts //

    noInterrupts();

    // Set timer register flags //

    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;

    // Configure timer registers //

    // OCR1A = (16 * 10^6) / (hertz * prescalar) - 1

    OCR1A = 31250;            // Compare match register 16MHz/256/2Hz
    TCCR1B |= (1 << WGM11);   // CTC mode
    TCCR1B |= (1 << CS01);    // Prescaler
    TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt

    // Enable interrupts //

    interrupts();

}

void NeuroBoard::startCommunicaton(void) {

    // TODO: See documentation.

    // Set serial to analog reading value //

    Serial.begin(SERIAL_CAP);

    // Set bool to enable serial writing //
    
    communicate = true;

}

int NeuroBoard::getNewSample(void) {

    // TODO: Return the newest sample from the buffer.

    int value = buffer[tail]; // Can't just return this because tail is changed below //
    full = false;
    tail = (tail + 1) % BUFFER_SIZE;

    return value;

}

int NeuroBoard::getEnvelopeValue(void) {

    // TODO: Return envelope value from ISR.

    return envelopeValue;

}

void NeuroBoard::setChannel(const uint8_t& newChannel) {

    // TODO: Set channel to passed channel.

    if (!validAnalog(newChannel)) {
        channel = this->channels[newChannel];
    } else {
        channel = newChannel;
    }

}

void NeuroBoard::enableButtonPress(const uint8_t& button, void callback(), const unsigned int& interval=250) {

    // TODO: Call callback function when button is pressed.

    if (button == RED_BTN) {
        redButtonTrigger._button = button;
        redButtonTrigger.callback = callback;
        redButtonTrigger.interval = interval;
        redButtonSet = true;
    }

    if (button == WHITE_BTN) {
        whiteButtonTrigger._button = button;
        whiteButtonTrigger.callback = callback;
        whiteButtonTrigger.interval = interval;
        whiteButtonSet = true;
    }

}

void NeuroBoard::enableButtonLongPress(const uint8_t& button, const unsigned int& milliseconds, void callback()) {

    // TODO: Call callback function when button is held for X milliseconds.

    if (button == RED_BTN) {
        redLongButtonTrigger._button = button;
        redLongButtonTrigger.callback = callback;
        redLongButtonTrigger.interval = milliseconds; // Instead of creating new struct, just use same variable.
        redLongButtonSet = true;
    }

    if (button == WHITE_BTN) {
        whiteLongButtonTrigger._button = button;
        whiteLongButtonTrigger.callback = callback;
        whiteLongButtonTrigger.interval = milliseconds; // Instead of creating new struct, just use same variable.
        whiteLongButtonSet = true;
    }

}

void NeuroBoard::setTriggerOnEnvelope(const unsigned int& threshold, void callback(), const unsigned int& secondFactor=0) {

    // TODO: Call callback when passed threshold is met by envelope value.

    const int THRESHOLD_SCALE_FACTOR = threshold / 10;
    int secondThreshold = secondFactor;

    if (secondFactor == 0) {
        secondThreshold = threshold - THRESHOLD_SCALE_FACTOR;
    }

    if (envelopeValue >= threshold) {
        if (!this->thresholdMet) {
            this->thresholdMet = true;
            callback();
        }
    } else {
        if (envelopeValue <= secondThreshold) {
            this->thresholdMet = false;
        }
    }

}

bool NeuroBoard::wait(const unsigned int& milliseconds) {

    //TODO: Return true once delay is finished.

    unsigned long long ms = millis();
    bool done = (ms - this->previousMilliseconds) >= milliseconds;

    if (done)
        this->previousMilliseconds = ms;
    return done;
    
}

bool NeuroBoard::wait(const unsigned int& milliseconds, void callback()) {

    // TODO: Wait n milliseconds, then invoke callback.

    unsigned long ms = millis();
    bool done = (ms - this->previousMillisecondsCallback) >= milliseconds;

    if (done) {
        this->previousMillisecondsCallback = ms;
        callback();
    }
    return done;

}

/// PRIVATE FUNCTIONS ///
unsigned long debouncingMilliseconds = 0;
bool debounceWait(const unsigned int& interval) {

    // TODO: Set debounce wait for 250 milliseconds.

    unsigned long ms = millis();
    bool done = (ms - debouncingMilliseconds) >= interval;

    if (done) 
        debouncingMilliseconds = ms;
    return done;

}

bool NeuroBoard::validAnalog(const uint8_t& newChannel) {

    // TODO: Check for valid analog within bounds.

	return !(newChannel < 0 or (newChannel > 11 and newChannel < 18) or newChannel > 29);

}

/* ******************************************************* */
/** @author Stanislav Mircic **/
/** Simplified by Ben Antonellis **/

void NeuroBoard::writeLEDs(void) {

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

void NeuroBoard::writeLED(int led, bool state) {

    byte bitMask = BITMASK_ONE;
    if (!(0 < led < 7)) {
        return;
    }
    if (state) {
        _shiftRegState |= bitMask << (7 - led);
    } else {
        _shiftRegState &= ~(bitMask << (7 - led));
    }

    writeLEDs();

}

/* ******************************************************* */
