#include <Arduino.h>

#include "config.h"

#include "hall_controller.h"
#include "led_controller.h"

HallSensorController hallController(PIN_MAG1_LS, PIN_MAG2_LS, PIN_MAG3_LS);
LEDController ledController(PIN_LED_LS, PIN_LED_DATA, LED_COUNT);

void setup()
{
    hallController.begin();
    ledController.begin();
    pinMode(LED_BUILTIN, OUTPUT);

    // Serial.begin(115200);
    // delay(1000);
    // Serial.println("Hello, world!");
}

void loop()
{
    // hallController.begin();
    // delay(100);
    // Read raw sensor data and serial print it
    // float sensorData[9];
    // hallController.readRaw(sensorData);
    double sensorData[9];
    hallController.read(sensorData);
    // Serial.print("Sensor 1: ");
    Serial.print(sensorData[0]);
    Serial.print(", ");
    Serial.print(sensorData[1]);
    Serial.print(", ");
    Serial.print(sensorData[2]);
    // Serial.print(" | Sensor 2: ");
    Serial.print(";  ");
    Serial.print(sensorData[3]);
    Serial.print(", ");
    Serial.print(sensorData[4]);
    Serial.print(", ");
    Serial.print(sensorData[5]);
    // Serial.print(" | Sensor 3: ");
    Serial.print(";  ");
    Serial.print(sensorData[6]);
    Serial.print(", ");
    Serial.print(sensorData[7]);
    Serial.print(", ");
    Serial.print(sensorData[8]);
    Serial.println();

    // hallController.printControlRegisters();

    delay(50);

    // // // put your main code here, to run repeatedly:
    // digitalWrite(LED_BUILTIN, HIGH);
    // // Serial.println("LED ON");
    // delay(200);
    // digitalWrite(LED_BUILTIN, LOW);
    // // Serial.println("LED OFF");
    // delay(200);
}