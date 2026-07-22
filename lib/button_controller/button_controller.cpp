#include "config.h"

#include "button_controller.h"

ButtonController::ButtonController(uint8_t pin_left, uint8_t pin_right)
: m_pinLeft(pin_left)
, m_pinRight(pin_right)
, m_buttonLeft(pin_left)
, m_buttonRight(pin_right)
{
    s_instance = this; // Set the static instance pointer to this instance
}

void ButtonController::handleEventBridge(ace_button::AceButton* button, uint8_t eventType, uint8_t buttonState)
{
    // This function is a static bridge to call the non-static member function
    // It assumes that the button pointer is valid and points to a ButtonController instance
    if (s_instance) {
        s_instance->handleButtonEvent(button, eventType);
    }
}

void ButtonController::handleButtonEvent(ace_button::AceButton* button, uint8_t eventType)
{
    uint32_t now = millis();

    // Handle left button events
    if (m_buttonLeftState == ButtonController::ButtonState::IDLE && button == &m_buttonLeft) {
        if (eventType == ace_button::AceButton::kEventPressed) {
            m_buttonLeftState = ButtonState::PRESSED;
        }
        else if (eventType == ace_button::AceButton::kEventReleased) {
            m_buttonLeftState = ButtonState::RELEASED;
        }
        else if (eventType == ace_button::AceButton::kEventClicked) {
            m_buttonLeftState = ButtonState::CLICKED;
        }
        else if (eventType == ace_button::AceButton::kEventDoubleClicked) {
            m_buttonLeftState = ButtonState::DOUBLE_CLICKED;
        }
        else if (eventType == ace_button::AceButton::kEventLongPressed) {
            m_buttonLeftState = ButtonState::PENDING_LONG_PRESS;
            m_leftPressTime = now; // Record the time when the long press started
        }
    }

    // Handle right button events
    if (m_buttonRightState == ButtonController::ButtonState::IDLE && button == &m_buttonRight) {
        if (eventType == ace_button::AceButton::kEventPressed) {
            m_buttonRightState = ButtonState::PRESSED;
        }
        else if (eventType == ace_button::AceButton::kEventReleased) {
            m_buttonRightState = ButtonState::RELEASED;
        }
        else if (eventType == ace_button::AceButton::kEventClicked) {
            m_buttonRightState = ButtonState::CLICKED;
        }
        if (eventType == ace_button::AceButton::kEventClicked) {
            m_buttonRightState = ButtonState::CLICKED;
        }
        else if (eventType == ace_button::AceButton::kEventDoubleClicked) {
            m_buttonRightState = ButtonState::DOUBLE_CLICKED;
        }
        else if (eventType == ace_button::AceButton::kEventLongPressed) {
            m_buttonRightState = ButtonState::PENDING_LONG_PRESS;
            m_rightPressTime = now; // Record the time when the long press started
        }
    }
}

void ButtonController::begin()
{
    pinMode(m_pinLeft, INPUT_PULLUP);
    pinMode(m_pinRight, INPUT_PULLUP);

    m_buttonLeft.init(m_pinLeft);
    m_buttonRight.init(m_pinRight);

    ace_button::ButtonConfig* buttonConfigLeft = m_buttonLeft.getButtonConfig();
    buttonConfigLeft->setFeature(ace_button::ButtonConfig::kFeatureClick);
    buttonConfigLeft->setFeature(ace_button::ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
    buttonConfigLeft->setFeature(ace_button::ButtonConfig::kFeatureDoubleClick);
    buttonConfigLeft->setFeature(ace_button::ButtonConfig::kFeatureLongPress);
    buttonConfigLeft->setEventHandler(handleEventBridge);

    ace_button::ButtonConfig* buttonConfigRight = m_buttonRight.getButtonConfig();
    buttonConfigRight->setFeature(ace_button::ButtonConfig::kFeatureClick);
    buttonConfigRight->setFeature(ace_button::ButtonConfig::kFeatureSuppressClickBeforeDoubleClick);
    buttonConfigRight->setFeature(ace_button::ButtonConfig::kFeatureDoubleClick);
    buttonConfigRight->setFeature(ace_button::ButtonConfig::kFeatureLongPress);
    buttonConfigRight->setEventHandler(handleEventBridge);
}

void ButtonController::update()
{
    m_buttonLeft.check();
    m_buttonRight.check();

    uint32_t now = millis();
    const uint32_t comboWindow = 500; // 500 ms threshold for combo detection

    if (m_buttonLeftState == ButtonState::PENDING_LONG_PRESS && m_buttonRightState == ButtonState::PENDING_LONG_PRESS) {
        // Both buttons are in pending long press state
        m_comboState = ComboState::LONG_PRESSED;
        m_buttonLeftState = ButtonState::IDLE;
        m_buttonRightState = ButtonState::IDLE;
    }
    else if (m_buttonLeftState == ButtonState::PENDING_LONG_PRESS && (now - m_leftPressTime > comboWindow)) {
        // Left button long press has exceeded the combo window without right button being pressed
        m_buttonLeftState = ButtonState::LONG_PRESSED;
    }
    else if (m_buttonRightState == ButtonState::PENDING_LONG_PRESS && (now - m_rightPressTime > comboWindow)) {
        // Right button long press has exceeded the combo window without left button being pressed
        m_buttonRightState = ButtonState::LONG_PRESSED;
    }
}

ButtonController::ButtonState ButtonController::getLeftButtonState()
{
    ButtonController::ButtonState state = m_buttonLeftState;

    if (state == ButtonController::ButtonState::PENDING_LONG_PRESS) {
        // Not sure yet if part of a combo, so don't reset state yet.
        // Instead return IDLE to indicate that the button is still being processed.
        return ButtonController::ButtonState::IDLE;
    }

    m_buttonLeftState = ButtonController::ButtonState::IDLE; // Reset state after reading
    return state;
}

ButtonController::ButtonState ButtonController::getRightButtonState()
{
    ButtonController::ButtonState state = m_buttonRightState;
    if (state == ButtonController::ButtonState::PENDING_LONG_PRESS) {
        // Not sure yet if part of a combo, so don't reset state yet.
        // Instead return IDLE to indicate that the button is still being processed.
        return ButtonController::ButtonState::IDLE;
    }

    m_buttonRightState = ButtonController::ButtonState::IDLE; // Reset state after reading
    return state;
}

ButtonController::ComboState ButtonController::getComboState()
{
    ButtonController::ComboState state = m_comboState;
    if (state != ButtonController::ComboState::IDLE) {
        m_comboState = ButtonController::ComboState::IDLE; // Reset state after reading
    }
    return state;
}