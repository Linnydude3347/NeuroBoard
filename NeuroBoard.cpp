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

// Button Wait Variables //

unsigned long redCount = 0;
unsigned long whiteCount = 0;
unsigned long redDebounceCount = 0;
unsigned long whiteDebounceCount = 0;

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

int redLongCalled = 0;
int whiteLongCalled = 0;

// ISR //

ISR (TIMER3_COMPA_vect) {

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

    if (redButtonTrigger.enabled) {
        if (digitalRead(redButtonTrigger._button)) {
            if (NeuroBoard::wait(redButtonTrigger.interval, redDebounceCount) and redButtonHeld == 0) {
                redButtonTrigger.callback();
            }
            redButtonHeld = 1;
        } else {
            redButtonHeld = 0;
        }

    }

    if (whiteButtonTrigger.enabled) {
        if (digitalRead(whiteButtonTrigger._button)) {
            if (NeuroBoard::wait(whiteButtonTrigger.interval, whiteDebounceCount) and whiteButtonHeld == 0) {
                whiteButtonTrigger.callback();
            }
            whiteButtonHeld = 1;
        } else {
            whiteButtonHeld = 0;
        }
    }

    if (redLongButtonTrigger.enabled) {
        if (digitalRead(redLongButtonTrigger._button)) {
            if (redLongButtonHeld) {
                if (NeuroBoard::wait(redLongButtonTrigger.interval, redCount)) {
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
        if (digitalRead(whiteLongButtonTrigger._button)) {
            if (whiteLongButtonHeld) {
                if (NeuroBoard::wait(whiteLongButtonTrigger.interval, whiteCount)) {
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

    // Check if envelope trigger is set

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

        // Turn ON relay if EMG is greater than threshold value (threshold is expressed in LED bar height units)
        if (servo.ledbarHeight > RELAY_THRESHOLD) {
            digitalWrite(RELAY_PIN, HIGH);
            delay(50);
        } else {
            digitalWrite(RELAY_PIN, LOW);
        }

        // Set new angle if enough time passed
        if (millis() - servo.oldTime > MINIMUM_SERVO_UPDATE_TIME) {
              // Calculate new angle for servo
              if (servo.currentFunctionality == OPEN_MODE) {  
                servo.analogReadings = constrain(servo.analogReadings, 40, servo.emgSaturationValue);
                servo.newDegree = map(servo.analogReadings, 40 , servo.emgSaturationValue, 190, 105); 
              } else {
                servo.analogReadings = constrain(servo.analogReadings, 120, servo.emgSaturationValue);
                servo.newDegree = map(servo.analogReadings, 120 , servo.emgSaturationValue, 105, 190);
              }
          
              // Check if we are in servo dead zone
              if (abs(servo.newDegree - servo.oldDegrees) > GRIPPER_MINIMUM_STEP) {
                 // Set new servo angle
                 servo.Gripper.write(servo.newDegree); 
              }
              servo.oldTime = millis();
              servo.oldDegrees = servo.newDegree;
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

    TCCR3A = 0;
    TCCR3B = 0;
    TCNT3 = 0;

    // Configure timer registers //

    // OCR1A = (16 * 10^6) / (hertz * prescalar) - 1

    OCR3A = 31250;            // Compare match register 16MHz/256/2Hz
    TCCR3B |= (1 << WGM11);   // CTC mode
    TCCR3B |= (1 << CS01);    // Prescaler
    TIMSK3 |= (1 << OCIE1A);  // enable timer compare interrupt

    // Enable interrupts //

    interrupts();

}

void NeuroBoard::startCommunicaton(void) {

    // Set bool to enable serial writing //
    
    communicate = true;

}

void NeuroBoard::startServo(void) {

    // Attach servo to board
    servo.Gripper.attach(SERVO_PIN);

    // Init button pins to input                         
    pinMode(RELAY_PIN, OUTPUT); 
    digitalWrite(RELAY_PIN, OFF);

    // Initialize all LED pins to output
    for(int i = 0; i < NUM_LED; i++){ 
        pinMode(ledPins[i], OUTPUT);
    }

    // Get current sensitivity
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];

    // Set servo enabled boolean
    servoEnabled = true;

}

void NeuroBoard::endServo(void) {

    // Detach servo
    servo.Gripper.detach();

    // Reset servo object to default values
    servo = NeuroServo();

}

void NeuroBoard::increaseSensitivity(void) {

    // Increment sensitivity index
    if (servo.lastSensitivitiesIndex != NUM_LED) {
        servo.lastSensitivitiesIndex++;
    }

    // Get current sensitivity value
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];

}

void NeuroBoard::decreaseSensitivity(void) {

    // Decrement sensitivity index
    if (servo.lastSensitivitiesIndex != 0) {
        servo.lastSensitivitiesIndex--;
    }

    // Get current sensitivity value
    servo.emgSaturationValue = servo.sensitivities[servo.lastSensitivitiesIndex];


}

void NeuroBoard::setServoDefaultPosition(const int& position) {

    servo.currentFunctionality = position;

}

void NeuroBoard::displayEMGStrength(void) {

    // Turn OFF all LEDs on LED bar
    for(int i = 0; i < NUM_LED; i++) {
        digitalWrite(servo.ledPins[i], OFF);
    }

    // Calculate what LEDs should be turned ON on the LED bar
    servo.analogReadings = constrain(servo.analogReadings, 30, servo.emgSaturationValue);
    servo.ledbarHeight = map(servo.analogReadings, 30, servo.emgSaturationValue, 0, NUM_LED);

    // Turn ON LEDs on the LED bar
    for(int i = 0; i < servo.ledbarHeight; i++) {
        digitalWrite(servo.ledPins[i], ON);
    }

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
        redButtonTrigger.set(button, callback, interval, true);
    }

    if (button == WHITE_BTN) {
        whiteButtonTrigger.set(button, callback, interval, true);
    }

}

void NeuroBoard::enableButtonPress(const uint8_t& button, void (*callback)(void)) {

    this->enableButtonPress(button, 250, callback);

}

void NeuroBoard::enableButtonLongPress(const uint8_t& button, const int& milliseconds, void (*callback)(void)) {

    if (button == RED_BTN) {
        redLongButtonTrigger.set(button, callback, milliseconds, true);
    }

    if (button == WHITE_BTN) {
        whiteLongButtonTrigger.set(button, callback, milliseconds, true);
    }

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, const int& secondFactor, void (*callback)(void)) {

    envelopeTrigger.set(threshold, secondFactor, callback, true, false);

}

void NeuroBoard::setTriggerOnEnvelope(const int& threshold, void (*callback)(void)) {

    this->setTriggerOnEnvelope(threshold, threshold - (threshold / 10), callback);

}

bool NeuroBoard::wait(const int& milliseconds, unsigned long& var) {
    unsigned long ms = millis();
    bool done = (ms - var) >= milliseconds;
    if (done)
        var = ms;
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