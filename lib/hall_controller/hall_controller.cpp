#include "config.h"

#include "hall_controller.h"

HallSensorController::HallSensorController(uint8_t sensor1PowerPin, uint8_t sensor2PowerPin, uint8_t sensor3PowerPin)
: m_sensor1PowerPin(sensor1PowerPin)
, m_sensor2PowerPin(sensor2PowerPin)
, m_sensor3PowerPin(sensor3PowerPin)
, m_sensor1(Wire, TLx493D_IIC_ADDR_A0_e)
, m_sensor2(Wire, TLx493D_IIC_ADDR_A0_e)
, m_sensor3(Wire, TLx493D_IIC_ADDR_A0_e)
{
}

void HallSensorController::begin()
{
    // Power pin configuration
    pinMode(m_sensor1PowerPin, OUTPUT);
    pinMode(m_sensor2PowerPin, OUTPUT);
    pinMode(m_sensor3PowerPin, OUTPUT);

    // All pins of initially set to LOW to power off the sensors
    powerOff(m_sensor1PowerPin);
    powerOff(m_sensor2PowerPin);
    powerOff(m_sensor3PowerPin);
    delay(5); // Wait for the sensors to power down

    // Initialize I2C communication
    Wire.begin();
    Wire.setClock(400000);
    // Wire.setClock(800000);

    // Power up and initialize sensors one by one to get unique I2C addresses
    powerOn(m_sensor1PowerPin);
    m_sensor1.init(true, false, false, true);
    m_sensor1.setIICAddress(TLx493D_IIC_ADDR_A2_e);
    m_sensor1.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor1.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor1.setSensitivity(TLx493D_SHORT_RANGE_e);
    delay(10); // Wait for the sensor to stabilize

    powerOn(m_sensor2PowerPin);
    m_sensor2.init(true, false, false, true);
    m_sensor2.setIICAddress(TLx493D_IIC_ADDR_A1_e);
    m_sensor2.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor2.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor2.setSensitivity(TLx493D_SHORT_RANGE_e);
    delay(10); // Wait for the sensor to stabilize

    powerOn(m_sensor3PowerPin);
    m_sensor3.init(true, false, false, true);
    m_sensor3.setIICAddress(TLx493D_IIC_ADDR_A0_e);
    m_sensor3.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor3.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor3.setSensitivity(TLx493D_SHORT_RANGE_e);
    delay(10); // Wait for the sensor to stabilize
}

void HallSensorController::powerOff(uint8_t pin)
{
    digitalWrite(pin, LOW);
}

void HallSensorController::powerOn(uint8_t pin)
{
    digitalWrite(pin, HIGH);
    delay(5);
}

void HallSensorController::readRaw(float out[9])
{
    // TODO: Original code uses getRawMagneticFieldAndTemperature,
    // but don't know why temperature is needed.
    // For now, just read magnetic field values.

    int16_t x, y, z, temperature;
    m_sensor1.getRawMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[0] = static_cast<float>(x);
    out[1] = static_cast<float>(y);
    out[2] = static_cast<float>(z);

    m_sensor2.getRawMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[3] = static_cast<float>(x);
    out[4] = static_cast<float>(y);
    out[5] = static_cast<float>(z);

    m_sensor3.getRawMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[6] = static_cast<float>(x);
    out[7] = static_cast<float>(y);
    out[8] = static_cast<float>(z);
}

void HallSensorController::read(double out[9])
{
    double x, y, z, temperature;
    m_sensor1.getMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[0] = x;
    out[1] = y;
    out[2] = z;
    m_sensor2.getMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[3] = x;
    out[4] = y;
    out[5] = z;
    m_sensor3.getMagneticFieldAndTemperature(&x, &y, &z, &temperature);
    out[6] = x;
    out[7] = y;
    out[8] = z;
}

void HallSensorController::printControlRegisters(uint8_t sensorID)
{
    ifx::tlx493d::TLx493D_A2B6* sensor = nullptr;
    if (sensorID == 1) {
        sensor = &m_sensor1;
    }
    else if (sensorID == 2) {
        sensor = &m_sensor2;
    }
    else if (sensorID == 3) {
        sensor = &m_sensor3;
    }
    else {
        Serial.println("Invalid sensor ID. Please provide a value between 1 and 3.");
    }
    if (sensor) {
        sensor->readRegisters();
        const uint8_t* map = sensor->getRegisterMap();
        const auto size = sensor->getRegisterMapSize();

        // Print 10h, 11h and 13h registers in binary format
        for (size_t i = 0; i < size; ++i) {
            if (i == 0x10 || i == 0x11 || i == 0x13) {
                Serial.print("Register 0x");
                Serial.print(i, HEX);
                Serial.print(": ");
                for (int bit = 7; bit >= 0; --bit) {
                    Serial.print((map[i] >> bit) & 1);
                }
                Serial.println();
            }
        }
    }
}

void HallSensorController::printControlRegisters()
{
    Serial.println("Sensor 1 Control Registers:");
    printControlRegisters(1);
    Serial.println("Sensor 2 Control Registers:");
    printControlRegisters(2);
    Serial.println("Sensor 3 Control Registers:");
    printControlRegisters(3);
}