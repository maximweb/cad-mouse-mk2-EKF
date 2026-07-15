

/*
    TEST FUNCTIONS
*/

// void test_DipoleModel()
// {
//     Serial.println("=== Testing DipoleModel with default magnetic moments (0.18 A*m^2) ===");
//     DipoleModel test_model;

//     float test_readings[9];
//     test_model.get_expected_readings(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, test_readings);
//     Serial.println("Expected readings with default magnetic moments (0.18 A*m^2):");
//     for (int i = 0; i < 3; ++i) {
//         Serial.print("Sensor ");
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.print(test_readings[i * 3 + 0], 6);
//         Serial.print(", ");
//         Serial.print(test_readings[i * 3 + 1], 6);
//         Serial.print(", ");
//         Serial.println(test_readings[i * 3 + 2], 6);
//     }
//     // Test at 12° RX
//     test_model.get_expected_readings(0.0f, 0.0f, 0.0f, 12.0f * 3.14159265f / 180.0f, 0.0f, 0.0f, test_readings);
//     Serial.println("Expected readings with default magnetic moments (0.18 A*m^2) at 12° RX:");
//     for (int i = 0; i < 3; ++i) {
//         Serial.print("Sensor ");
//         Serial.print(i);
//         Serial.print(": ");
//         Serial.print(test_readings[i * 3 + 0], 6);
//         Serial.print(", ");
//         Serial.print(test_readings[i * 3 + 1], 6);
//         Serial.print(", ");
//         Serial.println(test_readings[i * 3 + 2], 6);
//     }

//     Serial.println("=== DipoleModel test complete ===");
// }

// void test_calibration()
// {
//     Serial.println("=== Testing Calibration ===");
//     // Create a DipoleModel instance with known parameters
//     DipoleModel test_model;
//     float true_magnetic_moments[3];
//     float true_offsets[6];
//     test_model.get_magnetic_moments(true_magnetic_moments);
//     test_model.get_offsets(true_offsets);

//     // Collect sensor readings
//     Serial.println("Collecting sensor readings for calibration...");
//     Calibration::reset_calibration_data();
//     for (int i = 0; i < 50; ++i) {
//         float rawSensorData[9];
//         hallController.read(rawSensorData);
//         Calibration::add_sample(rawSensorData);
//         delay(20); // Wait 20 ms between samples
//     }
//     Serial.println("Sensor readings collection complete:");
//     float means[9];
//     float stds[9];
//     Calibration::get_current_means(means);
//     Calibration::get_current_stds(stds);
//     for (int i = 0; i < 9; ++i) {
//         uint8_t sensor_index = i / 3;
//         Serial.print("Sensor ");
//         Serial.print(sensor_index);
//         Serial.print(" Axis ");
//         Serial.print(i % 3);
//         Serial.print(" Mean: ");
//         Serial.print(means[i], 3);
//         Serial.print(", Std: ");
//         Serial.println(stds[i], 3);
//     }

//     // Perform calibration
//     float fitted_magnetic_moments[3];
//     float fitted_offsets[6];
//     bool calibration_success = Calibration::compute_calibration(fitted_magnetic_moments, fitted_offsets, test_model);
//     if (calibration_success) {
//         Serial.println("Calibration successful!");
//         Serial.print("Fitted magnetic moments: ");
//         for (int i = 0; i < 3; ++i) {
//             Serial.print(fitted_magnetic_moments[i], 6);
//             if (i < 2)
//                 Serial.print(", ");
//         }
//         Serial.println();
//         Serial.print("Fitted offsets: ");
//         for (int i = 0; i < 6; ++i) {
//             Serial.print(fitted_offsets[i], 6);
//             if (i < 5)
//                 Serial.print(", ");
//         }
//         Serial.println();
//     }
//     else {
//         Serial.println("Calibration failed.");
//     }

//     Serial.println("=== Calibration Test Complete ===");
// }

// void test_button_controller()
// {
//     buttonController.update();

//     switch (buttonController.getLeftButtonState()) {
//         case ButtonController::ButtonState::CLICKED:
//             Serial.println("Left button clicked");
//             ledController.queue_blinking_animation(0xFF0000, 300, 300, 1); // blink red once
//             break;
//         case ButtonController::ButtonState::DOUBLE_CLICKED:
//             Serial.println("Left button double clicked");
//             ledController.queue_pulse_animation(0xFF6600, 1000, 2); // pulse orange twice
//             break;
//         case ButtonController::ButtonState::LONG_PRESSED:
//             Serial.println("Left button long pressed");
//             ledController.queue_solid_animation(0xFFFF00, 2000); // solid yellow
//             break;
//         default:
//             break;
//     }

//     switch (buttonController.getRightButtonState()) {
//         case ButtonController::ButtonState::CLICKED:
//             Serial.println("Right button clicked");
//             ledController.queue_blinking_animation(0x00FF00, 500, 300, 1); // blink green once
//             break;
//         case ButtonController::ButtonState::DOUBLE_CLICKED:
//             Serial.println("Right button double clicked");
//             ledController.queue_blinking_animation(0x00FFFF, 200, 400, 2); // blink cyan once
//             break;
//         case ButtonController::ButtonState::LONG_PRESSED:
//             Serial.println("Right button long pressed");
//             ledController.queue_spinner_animation(0x0000FF, 500, 2, true, LEDController::SubMode::SPINNER_FULL); // spin blue once
//             break;
//         default:
//             break;
//     }

//     switch (buttonController.getComboState()) {
//         case ButtonController::ComboState::LONG_PRESSED:
//             Serial.println("Both buttons long pressed");
//             ledController.queue_spinner_animation(0xFF00FF, 500, 3, false, LEDController::SubMode::SPINNER_TAIL); // full spin magenta 3 times
//             break;
//         default:

//             break;
//     }
// }