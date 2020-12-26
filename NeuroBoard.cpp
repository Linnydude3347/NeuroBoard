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
/**
 * Private function to wait for the amount of time passed
 * to the enableButtonPress function.
 * 
 * @param interval Milliseconds to wait until next press.
 * 
 * @return bool.
**/
bool debounceWait(const uint8_t& interval) {

    unsigned long ms = millis();
    bool done = (ms - debouncingMilliseconds) >= interval;

    if (done) 
        debouncingMilliseconds = ms;
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

unsigned int NeuroBoard::redButtonHoldCount = 0;
unsigned int NeuroBoard::whiteButtonHoldCount = 0;

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

        const uint8_t outputFrameBuffer[2] = { (reading >> 7) | 0x80, reading & 0x7F };

        Serial.write(outputFrameBuffer, 2);

    }

    // Check if buttons are enabled //

    if (NeuroBoard::redButtonTrigger.enabled) {

        if (digitalRead(NeuroBoard::redButtonTrigger._button)) {
            if (debounceWait(NeuroBoard::redButtonTrigger.interval)) {
                NeuroBoard::redButtonTrigger.callback();
            }
        }

    }

    if (NeuroBoard::whiteButtonTrigger.enabled) {

        if (digitalRead(NeuroBoard::whiteButtonTrigger._button)) {
            if (debounceWait(NeuroBoard::whiteButtonTrigger.interval)) {
                NeuroBoard::whiteButtonTrigger.callback();
            }
        }

    }

    if (NeuroBoard::redLongButtonTrigger.enabled) {

        NeuroBoard::redButtonHoldCount = digitalRead(NeuroBoard::redLongButtonTrigger._button) ? (NeuroBoard::redButtonHoldCount + 1) : 0;
        if (NeuroBoard::redButtonHoldCount == NeuroBoard::redLongButtonTrigger.interval) {
            NeuroBoard::redButtonHoldCount = 0;
            NeuroBoard::redLongButtonTrigger.callback();
        }

    }

    if (NeuroBoard::whiteLongButtonTrigger.enabled) {

        NeuroBoard::whiteButtonHoldCount = digitalRead(NeuroBoard::whiteLongButtonTrigger._button) ? (NeuroBoard::whiteButtonHoldCount + 1) : 0;
        if (NeuroBoard::whiteButtonHoldCount == NeuroBoard::whiteLongButtonTrigger.interval) {
            NeuroBoard::whiteButtonHoldCount = 0;
            NeuroBoard::whiteLongButtonTrigger.callback();
        }

    }

}

// PUBLIC METHODS //

void NeuroBoard::startMeasurements(void) {

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

    // Set serial to analog reading value //

    Serial.begin(SERIAL_CAP);

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

void NeuroBoard::enableButtonPress(const uint8_t& button, void (*callback)(void), const uint8_t& interval) {

    if (button == RED_BTN) {
        NeuroBoard::redButtonTrigger.set(button, callback, interval, true);
    }

    if (button == WHITE_BTN) {
        NeuroBoard::whiteButtonTrigger.set(button, callback, interval, true);
    }

}

void NeuroBoard::enableButtonPress(const uint8_t& button, void (*callback)(void)) {

    this->enableButtonPress(button, callback, 250);

}

void NeuroBoard::enableButtonLongPress(const uint8_t& button, const uint8_t& milliseconds, void (*callback)(void)) {

    if (button == RED_BTN) {
        NeuroBoard::redLongButtonTrigger.set(button, callback, milliseconds, true);
    }

    if (button == WHITE_BTN) {
        NeuroBoard::whiteLongButtonTrigger.set(button, callback, milliseconds, true);
    }

}

void NeuroBoard::setTriggerOnEnvelope(const uint8_t& threshold, void (*callback)(void), const uint8_t& secondFactor) {

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

void NeuroBoard::setTriggerOnEnvelope(const uint8_t& threshold, void (*callback)(void)) {

    this->setTriggerOnEnvelope(threshold, callback, threshold - (threshold / 10));

}

bool NeuroBoard::wait(const uint8_t& milliseconds) {

    unsigned long long ms = millis();
    bool done = (ms - this->previousMilliseconds) >= milliseconds;

    if (done) {
        this->previousMilliseconds = ms;
    }
    return done;
    
}

bool NeuroBoard::wait(const uint8_t& milliseconds, void (*callback)(void)) {

    unsigned long ms = millis();
    bool done = (ms - this->previousMillisecondsCallback) >= milliseconds;

    if (done) {
        this->previousMillisecondsCallback = ms;
        callback();
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
