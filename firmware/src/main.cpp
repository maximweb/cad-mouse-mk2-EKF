#include <Arduino.h>
#include <pico/multicore.h>

#include "config.h"

#include "button_controller.h"
#include "calibration.h"
#include "dipole_model.h"
#include "extended_kalman_filter.h"
#include "hall_controller.h"
#include "hid_controller.h"
#include "led_controller.h"
#include "normalization.h"
#include "state_machine.h"

#ifdef _MAIN_SERIAL_DEBUG
#include "helpers.h"
#endif

/*
    GLOBAL OBJECTS
*/

HallSensorController hallController = HallSensorController(PIN_MAG1_LS, PIN_MAG2_LS, PIN_MAG3_LS);
LEDController ledController = LEDController(PIN_LED_LS, PIN_LED_DATA, LED_COUNT);
DipoleModel dipoleModel = DipoleModel();
ExtendedKalmanFilter ekf = ExtendedKalmanFilter();
ButtonController buttonController = ButtonController(PIN_LEFT_BTN, PIN_RIGHT_BTN);
StateMachine stateMachine = StateMachine(ledController, dipoleModel);
HIDController hidController = HIDController();

/*
    SHARED DATA STRUCTURES FOR CORE 0 AND CORE 1 COMMUNICATION
*/

// Shared raw sensor data for Core 0 to Core 1 communication
struct RawSensorData {
    float rawData[9];      // 3 sensors, each with 3 axes (X, Y, Z)
    uint32_t timestamp_us; // Timestamp of the last update in microseconds
};
volatile RawSensorData sharedRawSensorData;

// Shared filtered sensor data for Core 1 to Core 0 communication
struct FilteredData {
    float x, y, z;       // Filtered translation data
    float rx, ry, rz;    // Filtered rotation data
    float vx, vy, vz;    // Filtered translation velocity data
    float vrx, vry, vrz; // Filtered rotation velocity data
    float dt;            // Time delta for the last update in seconds
};
volatile FilteredData sharedFilteredData;

/*
    THREAD-SAFE LAST FILTERED DATA STORAGE
    This is used to store the latest filtered data received from Core 1 in a thread-safe manner,
    so that it can be accessed in the main loop without race conditions.
*/

float latest_estimated_state[12] = {0.0f};

void setup()
{
    Serial.begin(115200);

    // Initialize HID controller for USB communication
    hidController.begin();

    // Initialize the hall sensor controller
    hallController.begin();

    // Initialize the LED controller
    ledController.begin();

    // Initialize the button controller
    buttonController.begin();

    // Initialize the state machine
    stateMachine.enter_BOOT();

    // Initialize filesystem and load calibration data if available
    if (!Calibration::initialize_filesystem()) {
        ledController.queue_blinking_animation(LED_CALIBRATION_FAILURE_COLOR, 200, 200, 1); // blink MAGENTA one time to indicate filesystem initialization failure
    }
}

void loop()
{
    const uint32_t now = millis();
    float rawSensorData[9];
    bool sensor_status;

    StateMachine::State current_state = stateMachine.get_state();

    switch (current_state) {
        case StateMachine::State::BOOT: {
            // Give some time for the system to stabilize, then transition to CHECK_SENSORS

            // Read raw sensor data from the hall sensors, do nothing with it just yet
            sensor_status = hallController.read(rawSensorData);

            stateMachine.handle_BOOT(now);
            break;
        }

        case StateMachine::State::CHECK_SENSORS: {
            // Check if the hall sensors are delivering valid data
            sensor_status = hallController.read(rawSensorData);
#ifdef _MAIN_SERIAL_DEBUG
            Helpers::print_raw_sensor_data(rawSensorData);
#endif

            stateMachine.handle_CHECK_SENSORS(sensor_status);
            break;
        }

        case StateMachine::State::SENSOR_ERROR: {
            // Failed to get hall sensor values, reattempt CHECK_SENSORS repeatedly after a delay
            if (now - stateMachine.get_last_state_change_time_ms() > SENSOR_RECONNECT_DELAY_MS) {
                hallController.begin(); // Reinitialize the hall sensor controller
                stateMachine.enter_CHECK_SENSORS();
            }
            break;
        }

        case StateMachine::State::CALIBRATION_FROM_FILE: {
            // Attempt to load calibration data from file
            stateMachine.handle_CALIBRATION_FROM_FILE();
            break;
        }

        case StateMachine::State::CALIBRATE_COLLECT: {
            // Collect raw sensor data for calibration

            // Handle completeness and timeout checks for calibration collection
            if (stateMachine.handle_CALIBRATE_COLLECT_partial(now)) {
                break;
            }

            // Read raw sensor data from the hall sensors and add it to the calibration samples
            if (now - stateMachine.get_last_calibration_sample_time_ms() >= CALIBRATION_SAMPLE_DELAY_MS) {
                sensor_status = hallController.read(rawSensorData);
                if (sensor_status) {
                    Calibration::add_sample(rawSensorData);
                    stateMachine.set_last_calibration_sample_time_ms(now);
                }
            }
            break;
        }

        case StateMachine::State::CALIBRATE_COMPUTE: {
            stateMachine.handle_CALIBRATE_COMPUTE();
            break;
        }

        case StateMachine::State::RUNNING_WITHOUT_CALIBRATION:
        case StateMachine::State::RUNNING: {
            // RUNNING state: Normal operation,
            // Check for Core 1 response and update the latest estimated state
            // read raw sensor data, send to Core 1 for processing
            // TODO: Send via HID to host computer

            // Check if Core 1 is idle
            if (multicore_fifo_rvalid()) {
                uint32_t response = multicore_fifo_pop_blocking();

                // Check Core 1 for available processed data
                if (response == 3) {
                    // Core 1 has finished processing and is now idle
                    // Read the filtered pose from shared memory

                    stateMachine.set_last_filtered_data_received_time_ms(now);

                    latest_estimated_state[0] = sharedFilteredData.x;
                    latest_estimated_state[1] = sharedFilteredData.y;
                    latest_estimated_state[2] = sharedFilteredData.z;
                    latest_estimated_state[3] = sharedFilteredData.rx;
                    latest_estimated_state[4] = sharedFilteredData.ry;
                    latest_estimated_state[5] = sharedFilteredData.rz;
                    latest_estimated_state[6] = sharedFilteredData.vx;
                    latest_estimated_state[7] = sharedFilteredData.vy;
                    latest_estimated_state[8] = sharedFilteredData.vz;
                    latest_estimated_state[9] = sharedFilteredData.vrx;
                    latest_estimated_state[10] = sharedFilteredData.vry;
                    latest_estimated_state[11] = sharedFilteredData.vrz;

#ifdef _MAIN_SERIAL_DEBUG
                    // Print roundtrip time between consecutive readings->filtering->return
                    // Implies frequency of HID updates must be less than this
                    float dt_ms = sharedFilteredData.dt * 1e3;
                    Serial.print("Filter DT: ");
                    Serial.printf("%.3f", dt_ms);
                    Serial.println(" ms");
#endif

#ifdef _MAIN_SERIAL_DEBUG
                    // Print the latest estimated state for debugging
                    Helpers::print_estimated_state(latest_estimated_state);
                    // Condensed print for debugging
                    // Helpers::print_condensed_estimated_state(latest_estimated_state);
#endif
                }

                // Send new data to Core 1
                if (response == 1 || response == 3) {
                    // 1 = has never recieved any data
                    // 3 = has finished processing and is now idle

                    // Read sensor data and store in shared memory
                    sensor_status = hallController.read(rawSensorData);
                    if (sensor_status) {
                        // Store in shared memory for Core 1 to access
                        for (int i = 0; i < 9; ++i) {
                            sharedRawSensorData.rawData[i] = rawSensorData[i];
                        }
                        sharedRawSensorData.timestamp_us = micros();

                        __dmb();                         // Ensure memory is written before signaling Core 1
                        multicore_fifo_push_blocking(2); // 2 = Signal Core 1 to process new data
                    }
                }
            }

            // TODO: Send HID data at fixed intervals > dt of Kalman

            // Timeout check (no sensor readings) -> SENSOR_RECONNECT state
            if (now - stateMachine.get_last_filtered_data_received_time_ms() > RUNNING_STATE_READ_ERROR_TIMEOUT_MS) {
                stateMachine.enter_SENSOR_ERROR();
            }
            break;
        }
        default: {
            // Serial.println("Unknown state. Should not happen.");
            break;
        }
    }

    // Safe to access latest_estimated_state here for any other processing or output
    // For example, you could use it to update a display, send over serial, etc

    // Updates
    buttonController.update(); // Do first, so we can react to button presses immediately
    // test_button_controller();

    // Handle combo button states first
    ButtonController::ComboState combo_state = buttonController.getComboState();
    if (combo_state == ButtonController::ComboState::LONG_PRESSED) {
        stateMachine.enter_CALIBRATE_COLLECT();
    }

    static uint16_t buttons = 0; // Initialize hat switch value

    ButtonController::ButtonState left_button_state = buttonController.getLeftButtonState();
    ButtonController::ButtonState right_button_state = buttonController.getRightButtonState();

    if (left_button_state == ButtonController::ButtonState::PRESSED) {
        Serial.println("Left button pressed");
        buttons |= 0x0001; // Set bit 0 for left button press
        // x_test += 0.1f; // Increment x_test by 0.1
    }
    else if (left_button_state == ButtonController::ButtonState::RELEASED) {
        Serial.println("Left button released");
        buttons &= ~0x0001; // Clear bit 0 for left button release
    }
    if (right_button_state == ButtonController::ButtonState::PRESSED) {
        Serial.println("Right button pressed");
        buttons |= 0x0002; // Set bit 1 for right button press
    }
    else if (right_button_state == ButtonController::ButtonState::RELEASED) {
        Serial.println("Right button released");
        buttons &= ~0x0002; // Clear bit 1 for right button release
    }
    hidController.task();
    hidController.sendReport(latest_estimated_state, buttons);

    // LED controller update
    ledController.update();

    // Small delay just to keep your Serial monitor readable during testing
    // delay(10);
}

void setup1()
{
    const float initial_state[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f}; // Initial pose: x, y, z, rx, ry, rz
    ekf.init(initial_state, EKF_PROCESS_NOISE_STD, EKF_SENSOR_NOISE_STD);

    // Prime the engine: Push a fake complete token to kickstart Core 0's loop
    multicore_fifo_push_blocking(1); // 0 = Core 1 is idle and has never received any data
}

void loop1()
{
    // Wait for a signal from Core 0 to start processing
    uint32_t trigger = multicore_fifo_pop_blocking();

    // Bail out if Core 0 never sent any data
    if (trigger == 1) {
        return;
    }

    static uint32_t last_time_us = 0;
    static bool is_first_run = true;

    // On first run, we don't have a previous timestamp to calculate dt,
    // but we still can add data to the EKF and establish a baseline.
    if (is_first_run) {
        // Store timestamp of first data
        last_time_us = sharedRawSensorData.timestamp_us;
        is_first_run = false;

        // Update EKF with the first set of raw sensor data to establish a baseline
        float local_raw[9];
        for (int i = 0; i < 9; ++i) {
            local_raw[i] = sharedRawSensorData.rawData[i];
        }
        ekf.update(local_raw, dipoleModel);

        // Signal back to Core 0 that the first run is complete and dt can be established
        multicore_fifo_push_blocking(3);
        return;
    }

    // Calculate dt based on the timestamp of the latest raw sensor data
    float dt = (sharedRawSensorData.timestamp_us - last_time_us) * 1e-6f; // Convert microseconds to seconds
    last_time_us = sharedRawSensorData.timestamp_us;

    // Guard against massive timing spikes or clock hiccups
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.001f; // Fallback to a default 1ms step if timing fails
    }

    // Local copy to isolate processing memory
    float local_raw[9];
    for (int i = 0; i < 9; ++i) {
        local_raw[i] = sharedRawSensorData.rawData[i];
    }

    // Step the Kalman Filter math engine forward
    ekf.predict(dt);
    ekf.update(local_raw, dipoleModel);

    // Extract the filtered pose from the EKF state vector
    float estimated_state[12];
    ekf.get_state(estimated_state);

    // Apply normalization, deadzone, and curved isolation
    // This
    // - normalizes trans and rot to [-1, 1] and applies same factor to velocities
    // - applies deadzone to trans vector magnitude and rot vector magnitude
    // - applies curved isolation over all 6 DoF combined (power law + renormalization) and applies same factor to velocities
    float deadzone_normalized_state[12];
    Normalization::apply_normalization_deadzone_isolation(estimated_state, deadzone_normalized_state);

    // Store the processed state in shared memory for Core 0 to access
    sharedFilteredData.x = deadzone_normalized_state[0];
    sharedFilteredData.y = deadzone_normalized_state[1];
    sharedFilteredData.z = deadzone_normalized_state[2];
    sharedFilteredData.rx = deadzone_normalized_state[3];
    sharedFilteredData.ry = deadzone_normalized_state[4];
    sharedFilteredData.rz = deadzone_normalized_state[5];
    sharedFilteredData.vx = deadzone_normalized_state[6];
    sharedFilteredData.vy = deadzone_normalized_state[7];
    sharedFilteredData.vz = deadzone_normalized_state[8];
    sharedFilteredData.vrx = deadzone_normalized_state[9];
    sharedFilteredData.vry = deadzone_normalized_state[10];
    sharedFilteredData.vrz = deadzone_normalized_state[11];
    sharedFilteredData.dt = dt;

    __dmb(); // Ensure values are flushed to RAM

    // Once done processing, signal back to Core 0 that processing is complete
    multicore_fifo_push_blocking(3);
}