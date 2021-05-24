#pragma once

#include <Arduino.h>

class Heartbeater
{   
private:
    int _pin;
    uint8_t _onState;
    uint8_t _ledState;
    uint8_t _inBeatCount;
    unsigned long _beatInterval;
    unsigned long _onInterval = 30;
    unsigned long _offInterval = 150;
    unsigned long _inBeatInterval = 0;
    unsigned long _lastBeat;
    unsigned long _lastInBeat;
    unsigned long _beatCount = 0;
public:
    explicit Heartbeater(const int pin, uint8_t onState = HIGH, unsigned long beatIntervalMillis = 2000UL)
        : _pin(pin), _onState(onState), _beatInterval(beatIntervalMillis)
    {
        _ledState = !_onState;
        pinMode(_pin, OUTPUT);
    }

    void loop()
    {
        digitalWrite(_pin, _ledState); // each iteration of loop() will set the IO pins, even if they don't change, that's okay

        // Toggle back and forth between the two LEDs
        if ((millis() - _lastBeat) >= _beatInterval)
        {
            // time is up!
            _inBeatCount = 0;
            _lastBeat += _beatInterval;
            _beatCount++;
        }

        // Create the heartbeat effect
        if ((millis() - _lastInBeat) >= _inBeatInterval)
        {
            _lastInBeat = millis();
            if (_inBeatCount < 2)
            {
                _ledState = !_ledState;
                if (_ledState == _onState) {
                    _inBeatInterval = _onInterval;
                }
                else {
                    _inBeatCount++;
                    _inBeatInterval = _offInterval;
                }
            }
        }
    }
};