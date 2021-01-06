/**
 * NeuroBoard Implementation File.
 * 
 * @author Ben Antonellis
 * @date July 5th, 2020
**/

#include "NeuroBoard.hpp"

/* ******************************************************* */
/** @author Stanislav Mircic **/

#define SHIFT_LATCH_PIN B00000100                        // latch pin for shift register RCK - PB2
#define I_SHIFT_LATCH_PIN B11111011 
#define SHIFT_CLOCK_PIN B00000010                        // clock pin for shift register PB1
#define I_SHIFT_CLOCK_PIN B11111101 
#define SHIFT_DATA_PIN  B00001000                        // serial data pin for shift register SER - PB3
#define I_SHIFT_DATA_PIN  B11110111 
#define BITMASK_ONE B00000001

/* ******************************************************* */

/// PRIVATE FUNCTIONS ///

unsigned long debouncingMilliseconds = 0;
bool debounceWait(const uint8_t& interval) {
    unsigned long ms = millis();
    bool done = (ms - debouncingMilliseconds) >= interval;
    if (done) 
        debouncingMilliseconds = ms;
    return done;
}

unsigned long redCount = 0;
bool redHoldWait(const int& interval) {
    unsigned long ms = millis();
    bool done = (ms - redCount) >= interval;
    if (done)
        redCount = ms;
    return done;
}

unsigned long whiteCount = 0;
bool whiteHoldWait(const int& interval) {
    unsigned long ms = millis();
    bool done = (ms - whiteCount) >= interval;
    if (done)
        whiteCount = ms;
    return done;
}

/**
 * Determines if the passed analog is valid to use on the
 * Neuroduino Board. This is private because only our
 * function needs to use it.
 * 
 * @param newChannel The new analog channel to listen on.
 * 
 * @return bool.
**/
bool validAnalog(const uint8_t& newChannel) {

    return !(
        newChannel < 0 or                       // Any value less than 0 cannot be an analog.
        (newChannel > 11 and newChannel < 18)   // Allows integer input while also allowing A(N) input.
        or newChannel > 29                      // Anything bigger than 29 (A11) cannot be an analog.
    );

}

// Button Variables //

Button NeuroBoard::redButtonTrigger = Button();
Button NeuroBoard::whiteButtonTrigger = Button();
Button NeuroBoard::redLongButtonTrigger = Button();
Button NeuroBoard::whiteLongButtonTrigger = Button();

int NeuroBoard::decayRate = 1;

// Buffer Variables //

int* buffer = new int[BUFFER_SIZE];
int head = 0;
int tail = 0;
bool full = false;

// Envelope Value //

int envelopeValue;

// Serial variable, changed in startCommunication method //

bool communicate = false;
uint8_t NeuroBoard::channel = A0;

// Variables for button holding //

int redButtonHeld = 0;
int whiteButtonHeld = 0;

int redLongButtonHeld = 0;
int whiteLongButtonHeld = 0;

// ISR //

ISR (TIMER1_COMPA_vect) {

    // Get reading from analog //

    int reading = analogRead(NeuroBoard::channel);

    // Calculate envelope value here //

    envelopeValue = (reading >= envelopeValue) ? reading : envelopeValue - NeuroBoard::decayRate;

    // Place new reading in buffer //

    buffer[head] = reading;

    if (full) {
        tail = (tail + 1) % BUFFER_SIZE;
    }
    head = (head + 1) % BUFFER_SIZE;
    full = head == tail;

    // Check if communication is turned on //

    if (communicate) {

        //const uint8_t outputFrameBuffer[2] = { (reading >> 7) | 0x80, reading & 0x7F };

        //Serial.write(outputFrameBuffer, 2);

    }

    // Check if buttons are enabled //

    if (NeuroBoard::redButtonTrigger.enabled) {
        if (digitalRead(NeuroBoard::redButtonTrigger._button)) {
            if (debounceWait(NeuroBoard::redButtonTrigger.interval) and redButtonHeld == 0) {
                NeuroBoard::redButtonTrigger.callback();
            }
            redButtonHeld = 1;
        } else {
            redButtonHeld = 0;
        }

    }

    if (NeuroBoard::whiteButtonTrigger.enabled) {
        if (digitalRead(NeuroBoard::whiteButtonTrigger._button)) {
            if (debounceWait(NeuroBoard::whiteButtonTrigger.interval) and whiteButtonHeld == 0) {
                NeuroBoard::whiteButtonTrigger.callback();
            }
            whiteButtonHeld = 1;
        } else {
            whiteButtonHeld = 0;
        }
    }

    if (NeuroBoard::redLongButtonTrigger.enabled) {

        if (digitalRead(NeuroBoard::redLongButtonTrigger._button)) {
            if (redHoldWait(NeuroBoard::redLongButtonTrigger.interval) and redLongButtonHeld == 1) {
                NeuroBoard::redLongButtonTrigger.callback();
                redLongButtonHeld = 0;
            }
            redLongButtonHeld = 1;
        } else {
            redLongButtonHeld = 0;
        }

    }

    if (NeuroBoard::whiteLongButtonTrigger.enabled) {

        if (digitalRead(NeuroBoard::whiteLongButtonTrigger._button)) {
            if (redHoldWait(NeuroBoard::whiteLongButtonTrigger.interval) and whiteLongButtonHeld == 1) {
                NeuroBoard::whiteLongButtonTrigger.callback();
                whiteLongButtonHeld = 0;
            }
            whiteLongButtonHeld = 1;
        } else {
            whiteLongButtonHeld = 0;
        }

    }

}

// PUBLIC METHODS //

void NeuroBoard::startMeasurements(void) {

    // Start Serial //

    Serial.begin(SERIAL_CAP);

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

    // Set bool to enable serial writing //
    
    communicate = true;

}

int NeuroBoard::getNewSample(void) {

    int value = buffer[tail]; // Can't just return this because tail is changed below //
    full = false;
    tail = (tail + 1) % BUFFER_SIZE;

    return value;

}

int NeuroBoard::getEnvelopeValue(void) {

    return envelopeValue;

}

void NeuroBoard::setChannel(const uint8_t& newChannel) {

    if (!validAnalog(newChannel)) {
        NeuroBoard::channel = this->channels[newChannel];
    } else {
        NeuroBoard::channel = newChannel;
    }

}

void NeuroBoard::setDecayRate(const int& rate) {

    // Check to ensure positive input, some users may interpret decay rate
    // as a negative value. This prevents that mistake.

    NeuroBoard::decayRate = (rate < 0) ? abs(rate) : rate;

}

void NeuroBoard::enableButtonPress(const uint8_t& button, const int& interval, void (*callback)(void)) {

    if (button == RED_BTN) {
        NeuroBoard::redButtonTrigger.set(button, callback, interval, true);
    }

    if (button == WHITE_BTN) {
        NeuroBoard::whiteButtonTrigger.set(button, callback, interval, true);
    }

}

void NeuroBoard::enableButtonPress(const uint8_t& button, void (*callback)(void)) {

    this->enableButtonPress(button, 250, callback);

}

void NeuroBoard::enableButtonLongPress(const uint8_t& button, const int& milliseconds, void (*callback)(void)) {

    if (button == RED_BTN) {
        NeuroBoard::redLongButtonTrigger.set(button, callback, milliseconds, true);
    }

    if (button == WHITE_BTN) {
        NeuroBoard::whiteLongButtonTrigger.set(button, callback, milliseconds, true);
    }

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, const int& secondFactor, void (*callback)(void)) {

    if (envelopeValue >= threshold) {
        if (!this->thresholdMet) {
            this->thresholdMet = true;
            callback();
        }
    } else {
        if (envelopeValue <= secondFactor) {
            this->thresholdMet = false;
        }
    }

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, void (*callback)(void)) {

    this->setTriggerOnEnvelope(threshold, threshold - (threshold / 10), callback);

}

bool NeuroBoard::wait(const int& milliseconds) {

    unsigned long long ms = millis();
    bool done = (ms - this->previousMilliseconds) >= milliseconds;

    if (done) {
        this->previousMilliseconds = ms;
    }
    return done;

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

void NeuroBoard::writeLED(const int& led, const bool& state) {

    const int LED = ledPins[led];

    byte bitMask = BITMASK_ONE;
    if (!(0 < LED < 7)) {
        return;
    }
    if (state) {
        _shiftRegState |= bitMask << (7 - LED);
    } else {
        _shiftRegState &= ~(bitMask << (7 - LED));
    }

    writeLEDs();

}

/* ******************************************************* */