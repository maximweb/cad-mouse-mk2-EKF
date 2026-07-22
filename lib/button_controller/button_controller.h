#pragma once

#include <Arduino.h>

#include <AceButton.h>

class ButtonController {
public:
    enum class ButtonState {
        IDLE,
        PRESSED,
        CLICKED,
        RELEASED,
        DOUBLE_CLICKED,
        PENDING_LONG_PRESS,
        LONG_PRESSED,
        // REPEAT_PRESSED,
    };

    enum class ComboState {
        IDLE,
        LONG_PRESSED,
    };

    ButtonController(uint8_t pin_left, uint8_t pin_right);
    void begin();
    void update();
    ComboState getComboState();
    ButtonState getLeftButtonState();
    ButtonState getRightButtonState();

private:
    static inline ButtonController* s_instance = nullptr; // Static instance pointer for event handling

    uint8_t m_pinLeft;
    uint8_t m_pinRight;

    ace_button::AceButton m_buttonLeft;
    ace_button::AceButton m_buttonRight;

    ButtonState m_buttonLeftState{ButtonState::IDLE};
    ButtonState m_buttonRightState{ButtonState::IDLE};
    ComboState m_comboState{ComboState::IDLE};

    uint32_t m_leftPressTime{0};
    uint32_t m_rightPressTime{0};

    static void handleEventBridge(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState);
    void handleButtonEvent(ace_button::AceButton* button, uint8_t eventType);
};