/*
  Read_C6x0_Errors

  This example demonstrates how to read and interpret error codes from a C6x0 motor.
  It does not send any commands to the motor. Instead, it continuously listens for
  motor feedback on the CAN bus and continuously prints the motor's status, including the error code, to the Serial Monitor.

  This can be useful for diagnosing issues with a motor, such as a disconnected cable,
  overheating, or a stalled condition.
*/

// ▼▼▼ Select Board ▼▼▼ Uncomment only the board you are using
// #define USE_BOARD_ARDUINO_R4
// #define USE_BOARD_PICO
// #define USE_BOARD_ESP32

#if defined(USE_BOARD_ARDUINO_R4)
#include <Arduino_CAN.h>

#elif defined(USE_BOARD_ESP32)
#include <ESP32_TWAI.h>  // https://github.com/eyr1n/ESP32_TWAI
const gpio_num_t CAN_TX_PIN = 22;
const gpio_num_t CAN_RX_PIN = 21;

#elif defined(USE_BOARD_PICO)
#include <RP2040PIO_CAN.h>  //https://github.com/eyr1n/RP2040PIO_CAN
const uint32_t CAN_TX_PIN = 0;  // Pins do not need to be consecutive
const uint32_t CAN_RX_PIN = 1;  // Combinations like GP1 and GP3 also work

#else
#error "Board not selected. Please enable one of USE_BOARD_... at the top of the file."

#endif

#include <C6x0.h>  // Call after CAN library

// 1. Create a manager and the motor object you want to monitor.
C6x0Manager manager(&CAN);
C6x0<C6x0Type::C610> motor(manager, C6x0Id::ID_5);

// Function to convert error code enum to a human-readable string
const char* getErrorCodeString(C6x0ErrorCode code) {
  switch (code) {
    case C6x0ErrorCode::NO_ERROR:
      return "NO_ERROR";
    case C6x0ErrorCode::MOTOR_CHIP_ACCESS_FAILURE:
      return "MOTOR_CHIP_ACCESS_FAILURE";
    case C6x0ErrorCode::MSC_SUPPLY_OVER_VOLTAGE:
      return "MSC_SUPPLY_OVER_VOLTAGE";
    case C6x0ErrorCode::THREE_PHASE_CABLE_NOT_CONNECTED:
      return "THREE_PHASE_CABLE_NOT_CONNECTED";
    case C6x0ErrorCode::POSITION_SENSOR_SIGNAL_LOST:
      return "POSITION_SENSOR_SIGNAL_LOST";
    case C6x0ErrorCode::MOTOR_TEMPERATURE_CRITICAL:
      return "MOTOR_TEMPERATURE_CRITICAL";
    case C6x0ErrorCode::MOTOR_STALLED:
      return "MOTOR_STALLED";
    case C6x0ErrorCode::MOTOR_CALIBRATION_FAILED:
      return "MOTOR_CALIBRATION_FAILED";
    case C6x0ErrorCode::MOTOR_OVER_TEMPERATURE:
      return "MOTOR_OVER_TEMPERATURE";
    case C6x0ErrorCode::NO_CAN_MESSAGE:
      return "NO_CAN_MESSAGE";
    default:
      return "UNKNOWN_ERROR";
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);  // Wait for Serial port to be ready max 5s

  // 2. Initialize the CAN bus.
  bool can_ok = false;
#if defined(USE_BOARD_ARDUINO_R4)
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#elif defined(USE_BOARD_ESP32)
  can_ok = CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);
#elif defined(USE_BOARD_PICO)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#endif

  if (can_ok) {
    Serial.println("CAN bus initialized. Listening for motor errors...");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
}

void loop() {
  // 3. Call manager.update() to receive data from the motor.
  manager.update();

  // 4. Check the error code and print
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    last_print_time = millis();

    Serial.print("Error Code: ");
    Serial.print(static_cast<uint8_t>(motor.getErrorCode()));
    Serial.print(" (");
    Serial.print(getErrorCodeString(motor.getErrorCode()));
    Serial.print(")");
    Serial.print(", Degree: ");
    Serial.print(motor.getPositionDeg());
    Serial.print(", RPS: ");
    Serial.print(motor.getRps());
    Serial.println();
  }

  delay(1);
}
