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

#ifdef BOARD_RP2350
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 50 // 50 ms timeout for filtered data reception from Core 1 on RP2350
#else
#ifdef BOARD_RP2040
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 100 // 100 ms timeout for filtered data reception from Core 1 on RP2040
#else
#define RUNNING_STATE_READ_ERROR_TIMEOUT_MS 200 // 200 ms timeout for filtered data reception from Core 1 on unknown boards
#endif
#endif

#define RUNNING_STATE_INACTIVITY_TIMEOUT_MS 60000 // 60 seconds until LEDs turned off due to inactivity

// Button Controller
#define BUTTON_COMBO_WINDOW_MS 500 // Time window to detect combined long press of both buttons

// Dipole Model
#define DIPOLE_MODEL_MAGNETIC_MOMENT_DEFAULT 0.18f // Default magnetic moment for each of the three magnets in A*m^2

// Extended Kalman Filter
#define EKF_PROCESS_NOISE_STD 1.0f // Standard deviation for process noise
#define EKF_SENSOR_NOISE_STD 5.0f  // Standard deviation for sensor noise

// Jacobian reuse gating (M1 optimization)
// Jacobian recomputation is triggered if any of the configured criteria requires it.
#define EKF_JACOBIAN_REUSE_ENABLE 1
#define EKF_JACOBIAN_RECOMPUTE_INTERVAL 2       // Recompute every N updates (0 disables interval trigger)
#define EKF_JACOBIAN_INNOVATION_THRESHOLD 10.0f // Recompute when ||y||_2 exceeds this threshold (<= 0 disables threshold trigger)
#define EKF_JACOBIAN_MAX_REUSE_STREAK 2         // Force recompute after this many reuse-only updates

// Normalization, Deadzone, and Isolation
#define NORMALIZATION_X_MAX 1.7f  // Maximum translation in mm for normalization
#define NORMALIZATION_Y_MAX 1.7f  // Maximum translation in mm for normalization
#define NORMALIZATION_Z_MAX 1.5f  // Maximum translation in mm for normalization TODO: might need different (lower) MAX than MIN
#define NORMALIZATION_RX_MAX 6.5f // Maximum rotation in degrees for normalization
#define NORMALIZATION_RY_MAX 6.5f // Maximum rotation in degrees for normalization
#define NORMALIZATION_RZ_MAX 5.5f // Maximum rotation in degrees for normalization

#define DEADZONE_TRANSLATION_THRESHOLD 0.05f // Deadzone threshold for translation in normalized units (5%)
#define DEADZONE_ROTATION_THRESHOLD 0.05f    // Deadzone threshold for rotation in normalized units (5%)

#define ISOLATION_POWER 3.0f // Power for curved isolation (3.0f -> cubic isolation); while being a float, only 1, 2, 3, and 0.5 are optimized for RP2350 hardware. Other values will be slow.

// Calibration
#define CALIBRATION_DATA_STD_THRESHOLD 0.5f // Standard deviation threshold to accept collected raw data samples for calibration

#define CALIBRATION_FIT_MOMENT_BOUNDS 0.5f // bounds for magnetic moment fitting in A/m^2
#define CALIBRATION_FIT_MOMENT_MIN 0.05f   // minimum magnetic moment to accept in A/m^2

#define CALIBRATION_FIT_X_MIN -1.0f  // lower bound for x offset fitting in mm
#define CALIBRATION_FIT_X_MAX 1.0f   // upper bound for x offset fitting in mm
#define CALIBRATION_FIT_Y_MIN -1.0f  // lower bound for y offset fitting in mm
#define CALIBRATION_FIT_Y_MAX 1.0f   // upper bound for y offset fitting in mm
#define CALIBRATION_FIT_Z_MIN -1.0f  // lower bound for z offset fitting in mm
#define CALIBRATION_FIT_Z_MAX 1.0f   // upper bound for z offset fitting in mm
#define CALIBRATION_FIT_RX_MIN -1.0f // lower bound for rx offset fitting in degrees
#define CALIBRATION_FIT_RX_MAX 1.0f  // upper bound for rx offset fitting in degrees
#define CALIBRATION_FIT_RY_MIN -1.0f // lower bound for ry offset fitting in degrees
#define CALIBRATION_FIT_RY_MAX 1.0f  // upper bound for ry offset fitting in degrees
#define CALIBRATION_FIT_RZ_MIN -1.0f // lower bound for rz offset fitting in degrees
#define CALIBRATION_FIT_RZ_MAX 1.0f  // upper bound for rz offset fitting in degrees

// HID
#ifdef BOARD_RP2350
#define HID_REPORT_INTERVAL_MS 4 // 4 ms interval for sending HID reports (250 Hz) current Core 1 roundtrip time is ~2ms
#else
#ifdef BOARD_RP2040
#define HID_REPORT_INTERVAL_MS 12 // 12 ms interval for sending HID reports (83.3 Hz) current Core 1 roundtrip time is ~10.7ms
#else
#define HID_REPORT_INTERVAL_MS 20 // Default to 20 ms interval for sending HID reports (50 Hz) for unknown boards
#endif
#endif

// Debugging via Serial
// Uncomment any of the following lines to enable serial debug output for the corresponding module
// #define ENABLE_PERFORMANCE_PROFILING 1
// #define _MAIN_SERIAL_DEBUG 1
// #define _MAIN_SERIAL_PRINT_CORE1_DURATION 1
// #define _STATE_MACHINE_SERIAL_DEBUG 1
// #define _CALIBRATION_SERIAL_DEBUG 1
// #define _DIPOLE_MODEL_SERIAL_DEBUG 1
