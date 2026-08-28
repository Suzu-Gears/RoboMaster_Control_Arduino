English | [日本語](./README.ja.md)

[![Arduino Lint](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml/badge.svg)](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/Suzu-Gears/RoboMaster_Control_Arduino)

# RoboMaster_Control_Arduino

## Overview

This repository provides a library for handling DJI RoboMaster series Brushless DC Motors via CAN bus in the Arduino environment.

It supports:
- C6x0 Series
  - M2006 Motor (with C610/C610 v2 ESC)
  - M3508 Motor (with C620/C620 v2 ESC)
- GM6020 Series
  - GM6020 Voltage Control Mode
  - GM6020 Current Control Mode

## Features

This library allows for the mixed use of C6x0 series motors and GM6020 motors. It can also detect conflicts between feedback reception IDs and command transmission IDs, enabling safe motor management.

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

- **C6x0_Feedback**: Example for reading C6x0 series motor feedback (angle, RPM, error, etc.)
- **Mixed_Motors**: Example of handling C6x0 and GM6020 in a mixed environment.
- **C610_Speed_Control**: PI speed control of a C610 with anti-windup.
- **C610_Position_Control / C610_Position_Control2**: Cascade (position + velocity) PID position control of a C610.
- **C610_Step_Response / C610_Speed_Step_Response**: Step-response measurement for parameter identification.
- **speed_profile / speed_profile2** (RP2040 only): Tracking a pre-generated velocity profile in a 1 kHz timer interrupt (C610 / C620 variants).
- **ps3position** (RP2040 only): Velocity/distance control from a PS3 controller via SBDBT, with NeoPixel status display.

## Safety Notes

- **Check registration**: A motor constructor can fail to register (ID conflict, invalid ID, or `MaxMotors` exceeded). Check `manager.hasConflict()` and/or `motor.isValid()` in `setup()` and halt on failure. Getters on an invalid motor safely return default values; setters are ignored.
- **Feedback freshness**: Getters such as `getRpm()` keep returning the last received values even after a motor or the bus goes down. Use `motor.isFeedbackFresh(timeout_ms)` (or `getLastFeedbackMs()`) to detect feedback loss before trusting the values in a control loop.
- **Command watchdog**: By default, `transmit()` retransmits the last target forever, even if your control loop stops updating it. Call `manager.setCommandTimeout(ms)` to send 0 instead when a motor's target has not been refreshed within `ms` milliseconds.
- **Transmit blocking**: When the CAN TX queue is full, `transmit()` retries each frame (calling `yield()` between attempts) for up to 1 ms per frame (up to 5 frames), then drops it — the next `transmit()` supersedes it anyway. Tune this with `manager.setTxRetryTimeoutUs(us)` (0 = single attempt), especially when calling `transmit()` from a timer interrupt.
- **Interrupt safety**: The library has no internal locking. Call `update()`, `transmit()`, setters and getters from the same execution context, or guard them yourself. In particular, `getAccumPosition()` reads a 64-bit value that is not atomic on 32-bit MCUs.

## CAN ID Specifications

This library automatically sets the CAN ID according to the protocol based on the motor's ID. While ID conflicts can be detected by the library, the following tables are provided for design reference.

### C6x0 Series (C610/C620)

| Motor ID | Receive ID (Rx) | Transmit ID (Tx) | Transmit Data Bytes |
| :------: | :-------------: | :--------------: | :-------------------: |
| 1        | 0x201           | 0x200            | `DATA[0], DATA[1]`    |
| 2        | 0x202           | 0x200            | `DATA[2], DATA[3]`    |
| 3        | 0x203           | 0x200            | `DATA[4], DATA[5]`    |
| 4        | 0x204           | 0x200            | `DATA[6], DATA[7]`    |
| 5        | 0x205           | 0x1FF            | `DATA[0], DATA[1]`    |
| 6        | 0x206           | 0x1FF            | `DATA[2], DATA[3]`    |
| 7        | 0x207           | 0x1FF            | `DATA[4], DATA[5]`    |
| 8        | 0x208           | 0x1FF            | `DATA[6], DATA[7]`    |

### GM6020 (Voltage Control Mode)

| Motor ID | Receive ID (Rx) | Transmit ID (Tx) | Transmit Data Bytes |
| :------: | :-------------: | :--------------: | :-------------------: |
| 1        | 0x205           | 0x1FF            | `DATA[0], DATA[1]`    |
| 2        | 0x206           | 0x1FF            | `DATA[2], DATA[3]`    |
| 3        | 0x207           | 0x1FF            | `DATA[4], DATA[5]`    |
| 4        | 0x208           | 0x1FF            | `DATA[6], DATA[7]`    |
| 5        | 0x209           | 0x2FF            | `DATA[0], DATA[1]`    |
| 6        | 0x20A           | 0x2FF            | `DATA[2], DATA[3]`    |
| 7        | 0x20B           | 0x2FF            | `DATA[4], DATA[5]`    |
| 8(None)  | None            | None             | None                  |

### GM6020 (Current Control Mode)

| Motor ID | Receive ID (Rx) | Transmit ID (Tx) | Transmit Data Bytes |
| :------: | :-------------: | :--------------: | :-------------------: |
| 1        | 0x205           | 0x1FE            | `DATA[0], DATA[1]`    |
| 2        | 0x206           | 0x1FE            | `DATA[2], DATA[3]`    |
| 3        | 0x207           | 0x1FE            | `DATA[4], DATA[5]`    |
| 4        | 0x208           | 0x1FE            | `DATA[6], DATA[7]`    |
| 5        | 0x209           | 0x2FE            | `DATA[0], DATA[1]`    |
| 6        | 0x20A           | 0x2FE            | `DATA[2], DATA[3]`    |
| 7        | 0x20B           | 0x2FE            | `DATA[4], DATA[5]`    |
| 8(None)  | None            | None             | None                  |

## Error Codes

The following is a list of error codes that can be retrieved with `motor.getErrorCode()`.

| Code | Enum Name                         | Applies to | Description                                                     |
| :--: | :-------------------------------- | :--------: | :-------------------------------------------------------------- |
| 0    | `NO_ERROR`                        | C610, C620 | No abnormality (normal)                                         |
| 1    | `MOTOR_CHIP_ACCESS_FAILURE`       | C620       | Cannot access motor memory chip (during power-on self-test)     |
| 2    | `ESC_SUPPLY_OVER_VOLTAGE`         | C610, C620 | ESC supply voltage is too high (during power-on self-test)      |
| 3    | `THREE_PHASE_CABLE_NOT_CONNECTED` | C610, C620 | Three-phase cable to motor is not connected                     |
| 4    | `POSITION_SENSOR_SIGNAL_LOST`     | C610, C620 | Signal is lost on the 4-pin position sensor cable               |
| 5    | `MOTOR_TEMPERATURE_CRITICAL`      | C620       | Motor temperature is critical (e.g., >= 180°C)                  |
| 6    | `MOTOR_STALLED`                   | C610       | Motor is stalled                                                |
| 7    | `MOTOR_CALIBRATION_FAILED`        | C610, C620 | Motor calibration failed                                        |
| 8    | `MOTOR_OVER_TEMPERATURE`          | C620       | Motor is over temperature (e.g., >= 125°C)                      |
| 99   | `NO_CAN_MESSAGE`                  | C610, C620 | (Library-defined) No CAN message has been received yet          |
