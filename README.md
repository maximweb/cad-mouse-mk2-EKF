# CAD Mouse MK2 - Alternative Firmware

This is an alternative implementation for the awesome CAD Mouse MK2
([Code](https://github.com/sb-ocr/cad-mouse-mk2), [YouTube Video](https://youtu.be/62xlzGs8LXA), [Instructables](https://www.instructables.com/CAD-Mouse-MK2-a-6DoF-Space-Mouse-Using-Magnets), [preassembled PCB](https://ocrlab.myshopify.com/products/sensor-board-cad-mouse-mk2))

*I am not affiliated with the original author in any kind.*

I opted for a reimplementation rather than a fork because I wanted to learn how it works. Yet, several features closely follow the original implementation.

There is still some work do be done, but **I get consistent 250 Hz of HID data** despite heavy processing.

## Hardware

I chose a (pin compatible) SeeedStudio XIAO RP2350 instead of the original RP2040 because the former has

- generally more processing power
- hardware accelerated floating point capabilities

The PCB is a preassembled one I bought. Only hickup was a broken LED.
Hence my config.h contains setting for 7 instead of original 8 LEDs.

## Features

### Concept

The core idea was to improve the sensor processing / motion engine with

- a physics based dipole model
- in combination with an extended Kalman filter
- using dual-core capabilities of the MCU for the filtering

#### Dipole Physics Model

- The physics model represents each magnet by a dipole equation at the center of each magnet.
- The Hall-sensors are assumed to be in an equilateral triangle in a x-y-plane.
- The magnets are assumed to be in an equilateral triangle
  - in a x-y-plane above the sensors in neutral position
  - fixed to each other but transalted or rotated around the springs center
- [An external test recording raw readings at various known fixed positions revealed that a single dipole per magnet is sufficient to predict raw readings](https://github.com/sb-ocr/cad-mouse-mk2/issues/19#issuecomment-4987111997)

#### Kalman Filter and Postprocessing

- The sensor data are read in main loop and sent to Core1 for filtering
- an Extended Kalman filter based on the physics model then filter and predicts smooth x,y,z,rx,ry,rz state, as well as their (angular) velocities vx,vy,vz,vrx,vry,vrz
- Postprocessing does
  - normalize the translation (mm) and rotation (deg / rad) to [-1,1] -> still need to expose the limits
  - translation and rotation vectors are independently deadzones by calculating their vector magnitude, so we don't have any jitter in neutral position
  - to isolate movements, the combined state vector is cubed and renomarlized to pronounce movements of a dominant axis

### Calibration

Calibration is done in firmware by

- collecting and averaging sensor data
- fitting each magnets dipole magnetic moment via the dipole physics engine to best represent the Hall sensor magnetic field data
- fitting tiny x,y,z,rx,ry,rz offsets due to slight assembly inacuracies also using the dipole physics engine
- persisting the calibration results to file with LittleFS

Initial start triggers calibration and attempts to store it to LittleFS. Consecutive calibrations can be manually triggered by long press of both buttons simultaneously.

## ToDo

Open Points I still intent to address but especially reagrding the HID-Controller configuration, I appreciate any inputs.

- [ ] ButtonController: Remove unused states (we only need press, release, and longpress)
- [ ] HID-Controller
  - [ ] Generally not yet sure how to configure HID properly, to get the best experience. Needs input/investigation
  - [ ] Amend report to contain only two buttons
- [ ] Move all magic numbers to config.h


# CAD Mouse MK2 - Original

Watch the build video ↓

[<img src="./images/CAD_Mouse_MK2_Thumbnail.jpg">](https://youtu.be/62xlzGs8LXA)

This is the second iteration of my DIY CAD Mouse, rebuilt to behave like a real 6DoF controller. There are still some motion processing issues, but it's much better than the previous version. It uses a custom PCB with three magnetic sensors, a 3D printed spring, and a redesigned enclosure that is smaller and easier to build.

  Build instructions → [Instructables](https://www.instructables.com/CAD-Mouse-MK2-a-6DoF-Space-Mouse-Using-Magnets)

<sub>⚠️ There have been several comments raising concerns about the longevity of the PETG spring. If it does not last as expected, a revision of the knob design will be needed.</sub>

[![CC BY-NC-SA 4.0][cc-by-nc-sa-shield]][cc-by-nc-sa]

[![CC BY-NC-SA 4.0][cc-by-nc-sa-image]][cc-by-nc-sa]

[cc-by-nc-sa]: http://creativecommons.org/licenses/by-nc-sa/4.0/
[cc-by-nc-sa-image]: https://licensebuttons.net/l/by-nc-sa/4.0/88x31.png
[cc-by-nc-sa-shield]: https://img.shields.io/badge/License-CC%20BY--NC--SA%204.0-lightgrey.svg
