/**
 * NeuroBoard Header File.
 * 
 * @author Ben Antonellis
 * @date July 5th, 2020
**/

/** Possible Implementions
 * [ ] NOTE: Apply both button functions to one button.
 * [X] NOTE: Both buttons at the same time. (NOT POSSIBLE).
 * [ ] NOTE: Baud Rate to 9600, look into this.
 * [ ] NOTE: Possibly remove wait(time, callback) function.
 * [ ] NOTE: Add functionality for relays and servo - email stanislav for advice.
 * [ ] NOTE: Servo are 3 pins that connect to Neuroduino board. (Look on board too).
 * [ ] NOTE: Upon reaching certain threshold, activate the relay for Human To Human interface.
**/

/** Fixes
 * [ ] FIX: Fix enableButtonPress registering multiple times when held down.
 * [ ] FIX: enableButtonLongPress not registering - look into wait function.
 * [ ] FIX: Enable enableButtonPress and enableButtonLongPress on same button.
 * [ ] FIX: Fix setTriggerOnEnvelope only working once.
 * [ ] FIX: Make setTriggerOnEnvelope usable in setup function.
**/

#pragma once

#ifndef NEUROBOARD_HPP
#define NEUROBOARD_HPP

#include "Arduino.h"

// Defines //

#define RED_BTN DD4
#define WHITE_BTN DD7
#define ON HIGH
#define OFF LOW
#define BUFFER_SIZE 20
#define SERIAL_CAP 230400

#ifdef ARDUINO_AVR_UNO
    #define MAX_LEDS 6
#else // We must be dealing with a Leonardo
    #define MAX_LEDS 8
#endif

// Button Struct //

typedef struct button {

    uint8_t _button;
    void (*callback)(void);
    unsigned int interval;
    bool enabled;

    button(){};

    void set(uint8_t __button, void (*_callback)(void), const unsigned int& _interval, const bool& _enabled) {

        _button = __button;
        callback = _callback;
        interval = _interval;
        enabled = _enabled;

    }

} Button;

/**
 * Class for interacting with the Neuroduino Board.
**/
class NeuroBoard {

    public:

        // Static member variables //

        static uint8_t channel;

        static Button redButtonTrigger;
        static Button whiteButtonTrigger;
        static Button redLongButtonTrigger;
        static Button whiteLongButtonTrigger;

        static uint8_t redButtonHoldCount;
        static uint8_t whiteButtonHoldCount;

        static uint8_t decayRate;

        /**
         * Samples data to a circular buffer, and calculates envelope value 
         * all in the background. Sets pin modes for Stanislav's code.
         * 
         * - Usable in setup: true
         * - Usable in loop: false
         * 
         * @return void.
        **/
        void startMeasurements(void);

        /**
         * Enables periodic sending from buffer to SpikeRecorder. Waits for 
         * SpikeRecorder commands and responds.
         * 
         * - Usable in setup: true
         * - Usable in loop: false
         * 
         * @return void.
        **/
        void startCommunicaton(void);

        /**
         * Returns the last measured sample from the channel.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @return int - Last measured sample.
        **/
        int getNewSample(void);

        /**
         * Returns the envelope value of the channel.
         * 
         * For every incoming reading from the analog, if that reading isn't bigger
         * than the current envelope value, the envelope value is subtracted by one.
         * This ensures we don't have an envelope value higher than necessary.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @return int - Envelope value.
        **/
        int getEnvelopeValue(void);

        /**
         * Sets the current channel to listen on. Works with A0, A1, A2 etc and for
         * regular integer values. So if setChannel(0) is called, then A0 will be set.
         * If setChannel(7) is called, A7 will be set, and so on.
         * 
         * Boundries for regular integers:
         *      setChannel(n) where:
         *          0 <= n <= 11
         * Boundries for built in analogs:
         *      setChannel(n) where:
         *          A0 <= n <= A11
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param uint8_t Channel.
         * 
         * @return void.
        **/
        void setChannel(const uint8_t& channel);

        /**
         * Sets the decay rate for the setTriggerOnEnvelope function.
         * I.E, setDecayRate(5) will subtract envelopeValue by 5 every tick.
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param rate Integer representing the scale at which envelopeValue will be subtracted.
         * 
         * @return void.
        **/
        void setDecayRate(const uint8_t& rate);

        /**
         * Calls the passed function when the specified button is pressed.
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param button Which button to map the passed function to.
         * @param callback Function to call when button is pressed.
         * @param interval Delay between each button press to accept input.
         * 
         * @return void.
        **/
        void enableButtonPress(const uint8_t& button, void (*callback)(void), const uint8_t& interval);

        /**
         * Calls the passed function when the specified button is pressed.
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param button Which button to map the passed function to.
         * @param callback Function to call when button is pressed.
        **/
        void enableButtonPress(const uint8_t& button, void (*callback)(void));

        /**
         * Calls a function when a button is pressed for a long time.
         * See `longPressThreshold` for exact millisecond timing.
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param button Which button to map the passed function to.
         * @param milliseconds How many milliseconds the user should hold button for to execute function.
         * @param callback Function to call when button is held for X milliseconds.
         * 
         * @return void.
        **/
        void enableButtonLongPress(const uint8_t& button, const uint8_t& milliseconds, void (*callback)(void));

        /**
         * Calls the passed function when the envelope value is greater 
         * than the passed threshold.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param threshold Threshold for envelope value.
         * @param secondFactor Optional parameter for the second threshold the data must pass.
         * @param callback Function to call when threshold is reached.
         * 
         * @return void.
        **/
        void setTriggerOnEnvelope(const uint8_t& threshold, const uint8_t& secondFactor, void (*callback)(void));

        /**
         * Calls the passed function when the envelope value is greater
         * than the passed threshold.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param threshold Threshold for envelope value.
         * @param callback Function to call when threshold is reached.
         * 
         * @return void.
        **/
        void setTriggerOnEnvelope(const uint8_t& threshold, void (*callback)(void));

        /**
         * Custom delay function so our code can continue to run while the 
         * user wants to delay. This is non-blocking, so code will continue
         * to run.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param milliseconds How many milliseconds the user wants to delay.
         * 
         * @return void.
        **/
        bool wait(const uint8_t& milliseconds);

        /**
         * Custom delay function so our code can continue to run while the 
         * user wants to delay. This is non-blocking, so code will continue
         * to run.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param milliseconds How many milliseconds the user wants to delay.
         * @param callback Function to call every time delay is done.
         * 
         * @return void.
        **/
        bool wait(const uint8_t& milliseconds, void (*callback)(void));

        /* ******************************************************* */
        /** @author Stanislav Mircic **/

        void writeLED(const int& led, const bool& state);

        /* ******************************************************* */

    private:

        /* ******************************************************* */
        /** @author Stanislav Mircic **/

        void writeLEDs(void);
        void writeLEDs(byte outByte);
        byte _shiftRegState = 0;

        /* ******************************************************* */

        /**
         * NeuroServo object.
        **/
        NeuroServo servo;

        /**
         * Delay Variables.
        **/
        unsigned long previousMillisecondsCallback = 0;
        unsigned long previousMilliseconds = 0;

        /**
         * Counts how many milliseconds the button has been held.
        **/
        int buttonHoldCount = 0;

        /**
         * Flag for when threshold is reached to only call function once.
        **/
        bool thresholdMet = false;

        /**
         * Available channels to listen to. This is board specific.
         * Neuroduino (Arduino Leonardo).
        **/
        #ifdef ARDUINO_AVR_UNO
            int channels[8] = {A0, A1, A2, A3, A4, A5, A6, A7};
        #else // We must be dealing with a Leonardo
            int channels[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};
        #endif

        /**
         * LED pins for different boards.
        **/
        #ifdef ARDUINO_AVR_UNO
            int ledPins[6] = {8, 9, 10, 11, 12, 13};
        #else // We must be dealing with a Leonardo
            int ledPins[8] = {0, 1, 2, 3, 4, 5, 6, 7};
        #endif

};

// ============================== //
// ==========NERUOSERVO========== //
// ============================== //

/**
 * @author Stanislav Mircic.
 * @author Marcio Amorim.
 * 
 * - Updated and ported by Ben Antonellis.
**/

#include <Servo.h>

#define RELAY_PIN 3                         // Pin for relay that controls TENS device
#define RELAY_THRESHOLD 4                   // Defines sensitivity of relay
#define GRIPPER_STATE_BUTTON_PIN 4          // Pin for button that switches defult state of the gripper (opened/closed)
#define SERVO_PIN 2                         // Pin for servo motor
#define SENSITIVITY_BUTTON_PIN 7            // Pin for button that selects sesitivity
#define NUM_LED 6                           // Number of LEDs in LED bar
#define GRIPPER_MINIMUM_STEP 5              // 5 degree dead zone (used to avoid aiming oscilation)
#define OPEN_MODE 1                         // Default gripper state is opened
#define CLOSED_MODE 2                       // Default gripper state is closed
#define MINIMUM_SERVO_UPDATE_TIME 100       // Update servo position every 100ms

/**
 * Class for interacting with the Servo connected
 * to the Neuroduino Board.
**/
class NeuroServo {

    private:

        Servo gripper;                                              // Servo for gripper

        byte ledPins[6] = {8, 9, 10, 11, 12, 13};                   // Pins for LEDs in LED bar.
        int sensitivities[6] = {200, 350, 520, 680, 840, 1024};     // EMG saturation values (lower == closed, higher == open).
        int lastSensitivitiesIndex = 2;                             // Set initial sensitivity index.

        int emgSaturationValue = 0;                                 // Selected sensitivity/EMG saturation value.
        int analogReadings;                                         // Measured value for EMG.
        byte ledbarHeight = 0;                                      // Temporary variable for led bar height.

        unsigned long oldTime = 0;                                  // Timestamp of last servo angle update (milliseconds).
        int oldDegrees = 0;                                         // Old value of angle for servo.
        int newDegree;                                              // New value of angle for servo.

        unsigned long debouncerTimer = 0;                           // Timer for button debouncer.
        int gripperStateButtonValue = 0;                            // Temporary variable that stores state of the button.
        int userReleasedButton = 0;                                 // Flag that is used to avoid multiple button events when user holds button.

        int currentFunctionality = OPEN_MODE;                       // Current default position of claw.


    public:

        /**
         * Sets up the servo, inputs and outputs.
         * 
         * @return void.
        **/
        void enable();

        /**
         * Increases the sensitivity for the servo.
         * 
         * @return void.
        **/
        void increaseSensitivity();

        /**
         * Decreases the sensitivity for the servo.
        **/
        void decreaseSensitivity();

};

#endif // NEUROBOARD_HPP