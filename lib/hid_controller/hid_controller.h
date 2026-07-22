#pragma once

#include "config.h"

#include <Adafruit_TinyUSB.h>
#include <Arduino.h>

class HIDController {
public:
    struct Report {
        int16_t x;        ///< Delta x  movement of left analog-stick
        int16_t y;        ///< Delta y  movement of left analog-stick
        int16_t z;        ///< Delta z  movement of right analog-joystick
        int16_t rz;       ///< Delta Rz movement of right analog-joystick
        int16_t rx;       ///< Delta Rx movement of analog left trigger
        int16_t ry;       ///< Delta Ry movement of analog right trigger
        uint8_t hat;      ///< Buttons mask for currently pressed buttons in the DPad/hat
        uint32_t buttons; ///< Buttons mask for currently pressed buttons
    };

    bool begin();
    void task();
    void sendReport(float filtered_state[12], uint8_t hat, uint32_t buttons);

private:
    Adafruit_USBD_HID m_hid;

    uint32_t m_last_sent_time_ms{0}; // Timestamp of the last sent report
    Report m_report;
};