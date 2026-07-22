#include "config.h"

#include "hall_controller.h"

HallSensorController::HallSensorController(uint8_t sensor1PowerPin, uint8_t sensor2PowerPin, uint8_t sensor3PowerPin, bool fullRange)
: m_sensor1PowerPin(sensor1PowerPin)
, m_sensor2PowerPin(sensor2PowerPin)
, m_sensor3PowerPin(sensor3PowerPin)
, m_sensitivity(fullRange ? TLx493D_FULL_RANGE_e : TLx493D_SHORT_RANGE_e)
, m_scaleFactor(fullRange ? 7.7f : 15.4f)
, m_sensor1(Wire1, TLx493D_IIC_ADDR_A0_e)
, m_sensor2(Wire1, TLx493D_IIC_ADDR_A0_e)
, m_sensor3(Wire1, TLx493D_IIC_ADDR_A0_e)
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
    Wire1.begin();
    // Wire1.setClock(800000); // Datasheet recommends >= 800kHz for fast mode.
    Wire1.setClock(1000000); // Set I2C clock to 1 MHz (max) for faster communication

    // Power up and initialize sensors one by one to get unique I2C addresses
    powerOn(m_sensor1PowerPin);
    m_sensor1.init(true, false, false, true);
    m_sensor1.setIICAddress(TLx493D_IIC_ADDR_A2_e);
    m_sensor1.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor1.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor1.setSensitivity(m_sensitivity);
    delay(10); // Wait for the sensor to stabilize

    powerOn(m_sensor2PowerPin);
    m_sensor2.init(true, false, false, true);
    m_sensor2.setIICAddress(TLx493D_IIC_ADDR_A1_e);
    m_sensor2.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor2.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor2.setSensitivity(m_sensitivity);
    delay(10); // Wait for the sensor to stabilize

    powerOn(m_sensor3PowerPin);
    m_sensor3.init(true, false, false, true);
    m_sensor3.setIICAddress(TLx493D_IIC_ADDR_A0_e);
    m_sensor3.setPowerMode(TLx493D_FAST_MODE_e);
    // m_sensor3.setSensitivity(TLx493D_FULL_RANGE_e);
    m_sensor3.setSensitivity(m_sensitivity);
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

bool HallSensorController::readRaw(int16_t out[9])
{
    // TODO: Original code uses getRawMagneticFieldAndTemperature,
    // but don't know why temperature is needed.
    // For now, just read magnetic field values.

    // TODO: Original read functions are slow as they read all registers.
    // Hence, implement a faster read function that reads only the necessary registers
    // still checking parity.

    bool s1, s2, s3;
    int16_t x, y, z;

    // s1 = m_sensor1.getRawMagneticField(&x, &y, &z);
    s1 = readSingleSensorRawFast(m_sensor1.getI2CAddress() >> 1, &x, &y, &z);
    out[0] = x;
    out[1] = y;
    out[2] = z;

    // s2 = m_sensor2.getRawMagneticField(&x, &y, &z);
    s2 = readSingleSensorRawFast(m_sensor2.getI2CAddress() >> 1, &x, &y, &z);
    out[3] = x;
    out[4] = y;
    out[5] = z;

    // s3 = m_sensor3.getRawMagneticField(&x, &y, &z);
    s3 = readSingleSensorRawFast(m_sensor3.getI2CAddress() >> 1, &x, &y, &z);

    out[6] = x;
    out[7] = y;
    out[8] = z;

    return s1 && s2 && s3;
}

bool HallSensorController::read(float out[9])
{
    bool s;
    int16_t rawData[9];

    s = readRaw(rawData);

    for (int i = 0; i < 9; ++i) {
        out[i] = static_cast<float>(rawData[i]) / m_scaleFactor;
    }

    return s;
}

bool HallSensorController::readSingleSensorRawFast(uint8_t sensorAddress, int16_t* outX, int16_t* outY, int16_t* outZ)
{
    // 1-Byte read mode, request data directly from address without sending a register address first

    // Request first 7 bytes (data and diagnostics)
    if (Wire1.requestFrom(sensorAddress, static_cast<size_t>(7)) != 7)
        return false;

    // Read bytes
    uint8_t b[7];
    for (int i = 0; i < 7; ++i) {
        b[i] = Wire1.read();
    }

    // 1. XOR sum only the data registers (0x00 through 0x05)
    uint8_t parityCheck = 0;
    for (int i = 0; i < 6; ++i) {
        parityCheck ^= b[i];
    }

    // 2. Clear out the fuse/config flags from register 0x06,
    // leaving ONLY Bit 7 (Bus Parity) to add to the sum
    parityCheck ^= (b[6] & 0x80);

    // 3. Rapid bit-shift reduction to get the final 1-bit parity status
    parityCheck ^= parityCheck >> 4;
    parityCheck ^= parityCheck >> 2;
    parityCheck ^= parityCheck >> 1;

    // 4. Per your datasheet, the total parity must be ODD (lowest bit must be 1)
    if ((parityCheck & 0x01) == 0) {
        return false; // I2C transmission error detected! Drop the packet.
    }

    // 5. Parse the valid 12-bit signed data
    // BX LSB is in the upper nibble of b[4] (bits 7:4)
    *outX = static_cast<int16_t>((b[0] << 8) | (b[4] & 0xF0)) >> 4;

    // BY LSB is in the lower nibble of b[4] (bits 3:0) -> shift to alignment
    *outY = static_cast<int16_t>((b[1] << 8) | ((b[4] & 0x0F) << 4)) >> 4;

    // FIXED: BZ LSB is in the lower nibble of b[5] (bits 3:0) -> shift to alignment
    *outZ = static_cast<int16_t>((b[2] << 8) | ((b[5] & 0x0F) << 4)) >> 4;

    return true;

    return true;
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