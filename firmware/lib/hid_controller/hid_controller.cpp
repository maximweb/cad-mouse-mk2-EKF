#include "hid_controller.h"

namespace {
    const uint8_t hid_report_descriptor[] = {HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
                                             HID_USAGE(HID_USAGE_DESKTOP_JOYSTICK),
                                             HID_COLLECTION(HID_COLLECTION_APPLICATION),

                                             // 1. Report ID (Matches your m_hid.sendReport(1, ...))
                                             HID_REPORT_ID(1)

                                             // 2. Axes: 16-bit X, Y, Z, Rz, Rx, Ry
                                             HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
                                             HID_USAGE(HID_USAGE_DESKTOP_X),
                                             HID_USAGE(HID_USAGE_DESKTOP_Y),
                                             HID_USAGE(HID_USAGE_DESKTOP_Z),
                                             HID_USAGE(HID_USAGE_DESKTOP_RZ),
                                             HID_USAGE(HID_USAGE_DESKTOP_RX),
                                             HID_USAGE(HID_USAGE_DESKTOP_RY),
                                             HID_LOGICAL_MIN_N(0x8001, 2), // -32767
                                             HID_LOGICAL_MAX_N(0x7fff, 2), // 32767
                                             HID_REPORT_COUNT(6),
                                             HID_REPORT_SIZE(16),
                                             HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

                                             // 3. Hat Switch (D-Pad)
                                             HID_USAGE_PAGE(HID_USAGE_PAGE_DESKTOP),
                                             HID_USAGE(HID_USAGE_DESKTOP_HAT_SWITCH),
                                             HID_LOGICAL_MIN(1),
                                             HID_LOGICAL_MAX(8),
                                             HID_PHYSICAL_MIN(0),
                                             HID_PHYSICAL_MAX_N(315, 2), // Sets physical max to 315
                                             HID_REPORT_COUNT(1),
                                             HID_REPORT_SIZE(8),
                                             HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

                                             // 4. Buttons (32-bit map)
                                             HID_USAGE_PAGE(HID_USAGE_PAGE_BUTTON),
                                             HID_USAGE_MIN(1),
                                             HID_USAGE_MAX(32),
                                             HID_LOGICAL_MIN(0),
                                             HID_LOGICAL_MAX(1),

                                             // --- MACOS CRITICAL FIX: RESET THE PHYSICAL BOUNDS ---
                                             HID_PHYSICAL_MIN(0),
                                             HID_PHYSICAL_MAX(1),
                                             // -----------------------------------------------------

                                             HID_REPORT_COUNT(32),
                                             HID_REPORT_SIZE(1),
                                             HID_INPUT(HID_DATA | HID_VARIABLE | HID_ABSOLUTE),

                                             HID_COLLECTION_END};
}

bool HIDController::begin()
{
    if (!TinyUSBDevice.isInitialized()) {
        if (!TinyUSBDevice.begin(0)) {
            return false; // Failed to initialize TinyUSBDevice
        }
    }

    // Initialize the HID device with the report descriptor
    m_hid.setReportDescriptor(hid_report_descriptor, sizeof(hid_report_descriptor));
    m_hid.setPollInterval(HID_REPORT_INTERVAL_MS);
    m_hid.begin();

    // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
    if (TinyUSBDevice.mounted()) {
        TinyUSBDevice.detach();
        delay(10);
        TinyUSBDevice.attach();
    }

    m_report.x = 0;
    m_report.y = 0;
    m_report.z = 0;
    m_report.rz = 0;
    m_report.rx = 0;
    m_report.ry = 0;
    m_report.hat = 0;
    m_report.buttons = 0;

    return true;
}

void HIDController::task()
{
    TinyUSBDevice.task();
}

void HIDController::sendReport(float filtered_state[12], uint8_t hat, uint32_t buttons)
{
    // Send HID report

    // Skip sending if not mounted
    if (!TinyUSBDevice.mounted()) {
        return;
    }

    // Check if enough time has passed since the last report was sent
    const uint32_t now = millis();
    if (now - m_last_sent_time_ms < HID_REPORT_INTERVAL_MS) {
        return;
    }

    // Remote wakeup
    if (TinyUSBDevice.suspended() && buttons) {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        TinyUSBDevice.remoteWakeup();
    }

    if (!m_hid.ready()) {
        return; // HID device not ready to send report
    }

    // Prepare the report data
    m_report.hat = hat;         // Set hat switch value
    m_report.buttons = buttons; // Set buttons state

    // Mat the first 6 floats to the x, y, z, rz, rx, ry int8_t values
    // [-1, 1] range to [-32767, 32767] range
    m_report.x = static_cast<int16_t>(filtered_state[0] * 32767.f);
    m_report.y = static_cast<int16_t>(filtered_state[1] * 32767.f);
    m_report.z = static_cast<int16_t>(filtered_state[2] * 32767.f);
    m_report.rz = static_cast<int16_t>(filtered_state[3] * 32767.f);
    m_report.rx = static_cast<int16_t>(filtered_state[4] * 32767.f);
    m_report.ry = static_cast<int16_t>(filtered_state[5] * 32767.f);

    // report.x = random(-127, 128);
    // report.y = random(-127, 128);
    // report.z = random(-127, 128);
    // report.rz = random(-127, 128);
    // report.rx = random(-127, 128);
    // report.ry = random(-127, 128);
    // report.hat = random(0, 9);
    // report.buttons = random(0, 0xffff);

    // Serial.print("Sending HID report: ");
    // Serial.print("x: ");
    // Serial.println(m_report.x);
    // Serial.print(", y: ");
    // Serial.print(m_report.y);
    // Serial.print(", z: ");
    // Serial.print(m_report.z);
    // Serial.print(", rz: ");
    // Serial.print(m_report.rz);
    // Serial.print(", rx: ");
    // Serial.print(m_report.rx);
    // Serial.print(", ry: ");
    // Serial.print(m_report.ry);
    // Serial.print(", hat: ");
    // Serial.print(m_report.hat);
    // Serial.print(", buttons: ");
    // Serial.println(buttons, BIN);

    // static bool tic = false;
    // if (tic) {
    //     Serial.print(":");
    //     tic = false;
    // }
    // else {
    //     Serial.print(".");
    //     tic = true;
    // }

    m_hid.sendReport(1, &m_report, sizeof(m_report));
    m_last_sent_time_ms = now; // Update the timestamp of the last sent report
}