#pragma once

#include <Arduino.h>
#include <TLx493D_inc.hpp>
#include <Wire.h>

using namespace ifx::tlx493d;

class HallSensorController {
public:
    HallSensorController(uint8_t sensor1PowerPin, uint8_t sensor2PowerPin, uint8_t sensor3PowerPin);

    void begin();
    void readRaw(float out[9]);
    void read(double out[9]);

    void printControlRegisters();
    void printControlRegisters(uint8_t sensorID);

private:
    static void powerOff(uint8_t pin);
    static void powerOn(uint8_t pin);

    uint8_t m_sensor1PowerPin;
    uint8_t m_sensor2PowerPin;
    uint8_t m_sensor3PowerPin;

    ifx::tlx493d::TLx493D_A2B6 m_sensor1;
    ifx::tlx493d::TLx493D_A2B6 m_sensor2;
    ifx::tlx493d::TLx493D_A2B6 m_sensor3;
};