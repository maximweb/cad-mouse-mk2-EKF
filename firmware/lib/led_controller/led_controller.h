#pragma once

#include <Arduino.h>

#include <Adafruit_NeoPixel.h>

class LEDController {
public:
    LEDController(uint8_t pin_ls, uint8_t pin_data, uint8_t num_leds);
    void begin();
    void setSolid();
    //   void startSpinner(unsigned long color);
    //   void updateSpinner();
    void on();
    void off();

private:
    uint8_t m_pinLS;
    uint8_t m_pinData;
    uint8_t m_numLEDs;

    Adafruit_NeoPixel m_leds;
};