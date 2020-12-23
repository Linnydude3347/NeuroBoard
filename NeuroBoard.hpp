/**
 * NeuroBoard Header File.
 * 
 * @author Ben Antonellis
 * @date Jully 5th, 2020
**/

#pragma once

#ifndef NEUROBOARD_HPP
#define NEUROBOARD_HPP

#include "Arduino.h"

#define RED_BTN DD4
#define WHITE_BTN DD7
#define ON HIGH
#define OFF LOW
#define BUFFER_SIZE 20
#define MAX_LEDS 8
#define SERIAL_CAP 230400

struct button {

    uint8_t _button;
    void (*callback)(void);
    unsigned int interval;

    button(){};

};

typedef struct button Button;

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

        static bool redButtonSet;
        static bool whiteButtonSet;
        static bool redLongButtonSet;
        static bool whiteLongButtonSet;

        static unsigned int redButtonHoldCount;
        static unsigned int whiteButtonHoldCount;

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
         * @return void
        **/
        void setDecayRate(const int& rate);

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
        void enableButtonPress(const uint8_t& button, void (*callback)(void), const unsigned int& interval);

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
        void enableButtonLongPress(const uint8_t& button, const unsigned int& milliseconds, void (*callback)(void));

        /**
         * Calls the passed function when the envelope value is greater 
         * than the passed threshold.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param threshold Threshold for envelope value.
         * @param callback Function to call when threshold is reached.
         * @param secondFactor Optional parameter for the second threshold the data must pass.
         * 
         * @return void.
        **/
        void setTriggerOnEnvelope(const unsigned int& threshold, void (*callback)(void), const unsigned int& secondFactor);

        /**
         * Calls the passed function when the envelope value is greater
         * than the passed threshold.
         * 
         * @param threshold Threshold for envelope value.
         * @param callback Function to call when threshold is reached.
         * 
         * @return void.
        **/
        void setTriggerOnEnvelope(const unsigned int& threshold, void (*callback)(void));

        /**
         * Custom delay function so our code can continue to run while the 
         * user wants to delay.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param milliseconds How many milliseconds the user wants to delay.
         * 
         * @return void.
        **/
        bool wait(const unsigned int& milliseconds);

        /**
         * Custom delay function so our code can continue to run while the 
         * user wants to delay.
         * 
         * - Usable in setup: false
         * - Usable in loop: true
         * 
         * @param milliseconds How many milliseconds the user wants to delay.
         * @param callback Function to call every time delay is done.
         * 
         * @return void.
        **/
        bool wait(const unsigned int& milliseconds, void (*callback)(void));

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
         * Delay Variables.
        **/
        unsigned long previousMillisecondsCallback = 0;
        unsigned long previousMilliseconds = 0;

        /**
         * Counts how many milliseconds the button has been held
        **/
        int buttonHoldCount = 0;

        /**
         * Flag for when threshold is reached to only call function once.
        **/
        bool thresholdMet = false;

        /**
         * Available channels to listen to
        **/
        int channels[12] = {A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11};

};

#endif