/**
 * NeuroBoard Header File.
 * 
 * @author Ben Antonellis
 * @date July 5th, 2020
**/

/** Possible Implementions
 * [X] NOTE: Apply both button functions to one button. (NOT POSSIBLE).
 * [X] NOTE: Both buttons at the same time. (NOT POSSIBLE).
 * [X] NOTE: Baud Rate to 9600, look into this.
 * [X] NOTE: Possibly remove wait(time, callback) function.
 * [ ] NOTE: Add functionality for relays and servo - email stanislav for advice.
 * [X] NOTE: Servo are 3 pins that connect to Neuroduino board. (Look on board too).
 * [ ] NOTE: Upon reaching certain threshold, activate the relay for Human To Human interface.
**/

/** Fixes
 * [X] FIX: Fix enableButtonPress registering multiple times when held down.
 * [X] FIX: enableButtonLongPress not registering - look into wait function.
 * [X] FIX: Enable enableButtonPress and enableButtonLongPress on same button. (NOT POSSIBLE).
 * [ ] FIX: Fix setTriggerOnEnvelope only working once.
 * [ ] FIX: Make setTriggerOnEnvelope usable in setup function.
**/

#pragma once

#ifndef NEUROBOARD_HPP
#define NEUROBOARD_HPP

#include "Arduino.h"
#include <Servo.h>

// Defines //

#define RED_BTN DD4
#define WHITE_BTN DD7
#define ON HIGH
#define OFF LOW
#define BUFFER_SIZE 20
#define SERIAL_CAP 9600

#ifdef ARDUINO_AVR_UNO
    #define MAX_LEDS 6
#else // We must be dealing with a Leonardo
    #define MAX_LEDS 8
#endif

// Button Struct //

struct Button {

    uint8_t _button;
    void (*callback)(void);
    unsigned int interval;
    bool enabled;

    Button(){};

    void set(uint8_t __button, void (*_callback)(void), const int& _interval, const bool& _enabled) {

        _button = __button;
        callback = _callback;
        interval = _interval;
        enabled = _enabled;

    }

};

/**
 * Class for interacting with the Neuroduino Board.
**/
class NeuroBoard {

    public:

        // Static member variables //

        static uint8_t channel;
        static int decayRate;

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
        void setDecayRate(const int& rate);

        /**
         * Calls the passed function when the specified button is pressed.
         * 
         * - Usable in setup: true
         * - Usable in loop: true
         * 
         * @param button Which button to map the passed function to.
         * @param interval Delay between each button press to accept input.
         * @param callback Function to call when button is pressed.
         * 
         * @return void.
        **/
        void enableButtonPress(const uint8_t& button, const int& interval, void (*callback)(void));

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
        void enableButtonLongPress(const uint8_t& button, const int& milliseconds, void (*callback)(void));

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
        void setTriggerOnEnvelope(const int& threshold, const int& secondFactor, void (*callback)(void));

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
        void setTriggerOnEnvelope(const int& threshold, void (*callback)(void));

        /**
         * Custom delay function so our code can continue to run while the 
         * user wants to delay. This is non-blocking, so code will continue
         * to run.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param milliseconds How many milliseconds the user wants to delay.
         * @param var Variable to use to hold count.
         * 
         * @return void.
        **/
        static bool wait(const int& milliseconds, unsigned long& var);

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

#endif // NEUROBOARD_HPP
