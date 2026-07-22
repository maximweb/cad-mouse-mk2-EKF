// Uncomment to avoid HID output but return values over Serial for testing
#define DEVELOPMENT_MODE 1

// Pins as defined in the original project
#define PIN_RIGHT_BTN D0
#define PIN_LEFT_BTN D2
#define PIN_LED_DATA D3
#define PIN_LED_LS D1
#define PIN_MAG1_LS D10
#define PIN_MAG2_LS D9
#define PIN_MAG3_LS D8

// RGB LEDs
#define LED_COUNT 7
#define LED_BRIGHTNESS 100                             // 0 to 255
#define LED_BOOT_COLOR 0xFFFF00                        // Yellow
#define LED_ERROR_COLOR 0xFF0000                       // Red
#define LED_SUCCESS_COLOR 0x00FF00                     // Green
#define LED_CALIBRATION_COLOR 0x0000FF                 // Blue
#define LED_CALIBRATION_SUCCESS_COLOR 0x00FFFF         // Cyan
#define LED_CALIBRATION_FAILURE_COLOR 0xFF00FF         // Magenta
#define LED_RUNNING_COLOR 0xFFFFFF                     // White
#define LED_RUNNING_WITHOUT_CALIBRATION_COLOR 0xFF6600 // Orange

// State Machine
#define BOOT_DELAY_MS 1000
#define SENSOR_RECONNECT_DELAY_MS 1000
#define CALIBRATION_SAMPLE_COUNT 100
#define CALIBRATION_SAMPLE_DELAY_MS 20
#define CALIBRATION_SAMPLE_TIMEOUT_MS (CALIBRATION_SAMPLE_COUNT * CALIBRATION_SAMPLE_DELAY_MS * 5)
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 50

// HID
#define HID_REPORT_INTERVAL_MS 4 // 4 ms interval for sending HID reports (250 Hz) current Core 1 roundtrip time is ~2ms