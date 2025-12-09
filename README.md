English | [日本語](./README.ja.md)

[![Arduino Lint](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml/badge.svg)](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml)

# RoboMaster_Control_Arduino

## Overview

This repository contains a set of libraries for handling DJI RoboMaster series Brushless DC Motors via CAN bus in the Arduino environment. Strictly speaking, it consists of the following three libraries:

- C6x0 series library (for C610/C610 v2/C620/C620 v2 ESCs)
- GM6020 library (for GM6020 motors)
- Mixed-use library

Each is designed to be used for different purposes.

## Features

- **`C6x0.h`**: A simple library for controlling C610/C610 v2/C620/C620 v2 ESCs.
- **`GM6020.h`**: A simple library for controlling GM6020 motors.
- **`RoboMaster_Control.h`**: A library that can manage a mix of C6x0 ESCs and GM6020 motors. It detects CAN ID and transmission slot conflicts with compile-time checks (static_assert).

## Supported Hardware and Dependencies

This library is compliant with `arduino::HardwareCAN`. A separate library for CAN communication is required.

| Microcontroller Board                           | Required CAN Library                                    |
| ----------------------------------------------- | ------------------------------------------------------- |
| Arduino (Uno R4 WiFi / Uno R4 Minima / Nano R4) | Arduino_CAN (Built-in)                                  |
| Raspberry Pi Pico (RP2040 / RP2350)             | [RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN) |
| ESP32                                           | [ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)       |

**Note:** A CAN transceiver is required.

## Installation

### Manual Installation

#### Using "Install .ZIP Library" in Arduino IDE (Recommended)

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/releases/latest) as a `.zip` file.
2.  In the Arduino IDE, go to `Sketch > Include Library > Install .ZIP Library...`.
3.  Select the downloaded `.zip` file.
4.  Restart the Arduino IDE.

#### Manual Placement

1.  Download the latest release from the [GitHub repository](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/releases/latest) as a `.zip` file.
2.  Unzip the downloaded file.
3.  Place the unzipped folder into your Arduino libraries directory (e.g., `~/Documents/Arduino/libraries/`).
4.  Restart the Arduino IDE.

## Basic Usage

Refer to the samples in the `examples` folder for details.

- **C6x0_Multi**: Example of C6x0 series ESC control.
- **C6x0_Read_Errors**: Example of reading feedback/errors.
- **GM6020_Multi**: Example of GM6020 voltage/current mode control.
- **Mixed_Motors**: Example of handling C6x0 and GM6020 in a mixed environment.
- **Mixed_Motors_Macro**: An alternative version of `Mixed_Motors` that defines motor aliases using C macros.
- **Compile_Time_Check**: Example of CAN ID conflict detection (intentionally causes a compile error).