#include "config.h"

#include "led_controller.h"

LEDController::LEDController(uint8_t pin_ls, uint8_t pin_data, uint8_t num_leds)
: m_pinLS(pin_ls)
, m_pinData(pin_data)
, m_numLEDs(num_leds)
, m_leds(num_leds, pin_data, NEO_GRB + NEO_KHZ800)
{
}

void LEDController::begin()
{
    pinMode(m_pinLS, OUTPUT);
    digitalWrite(m_pinLS, LOW);

    m_leds.begin();
    m_leds.setBrightness(LED_BRIGHTNESS);
    m_leds.show();

    setSolid(); // Set initial state to solid color
    on();       // Turn on the LEDs
}

void LEDController::on()
{
    digitalWrite(m_pinLS, HIGH);
    delay(10); // Wait for the latch signal to stabilize
}

void LEDController::off()
{
    digitalWrite(m_pinLS, LOW);
    delay(10); // Wait for the latch signal to stabilize
}

void LEDController::setSolid()
{
    //   mode_ = Mode::Solid;
    //   color_ = toNeoColor(color);
    // setPower(true);
    // fillAll(color_);

    // Fill all LEDs with the specified color
    for (uint8_t i = 0; i < m_leds.numPixels(); ++i) {
        m_leds.setPixelColor(i, m_leds.Color(255, 0, 0));
    }

    m_leds.show();
}