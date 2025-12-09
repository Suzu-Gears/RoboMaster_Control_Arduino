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

#include <C6x0.h>  // Call after CAN library, otherwise api/HardwareCAN.h might not be found

// 1. Create a single manager for all C6x0 motors on the same CAN bus.
C6x0Manager manager(&CAN);

// 2. Create C6x0 objects for each motor you want to control.
// This example controls a C610 (ID 1) and a C620 (ID 5).
// Note: C610 and C620 have different current limits.
// C610: ±10A (±10000mA)
// C620: ±20A (±20000mA)
C6x0<C6x0Type::C610> motor1(manager, C6x0Id::ID_1);
C6x0<C6x0Type::C620> motor2(manager, C6x0Id::ID_5);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 5000);  // Wait for Serial port to be ready max 5s

  // 3. Initialize the CAN bus.
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
    Serial.println("CAN bus initialized successfully.");
  } else {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
}

void loop() {
  // 4. Call manager.update() to receive data from all motors.
  manager.update();

  // 5. Implement control logic for each motor.
  // This example makes motor1 spin at 10 RPS and motor2 at -10 RPS.
  const float kp = 100.0f;

  // Motor 1 control
  const float target_rps_1 = 10.0f;
  const float current_rps_1 = motor1.getRpm() / 60.0f;
  const float error_1 = target_rps_1 - current_rps_1;
  const float command_current_1 = kp * error_1;
  motor1.setCurrent(command_current_1);

  // Motor 2 control
  const float target_rps_2 = -10.0f;
  const float current_rps_2 = motor2.getRps();  // Using getRps() directly
  const float error_2 = target_rps_2 - current_rps_2;
  const float command_current_2 = kp * error_2;
  motor2.setCurrent(command_current_2);

  // 6. Call manager.transmit() to send commands to all motors.
  manager.transmit();

  // 7. Print motor data for debugging every 100ms.
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    last_print_time = millis();

    Serial.print("ID: 1");
    Serial.print(", AccumDegree: ");
    Serial.print(motor1.getAccumPositionDeg());
    Serial.print(", RPS: ");
    Serial.print(motor1.getRps());
    Serial.print(", TorqueCurrent: ");
    Serial.print(motor1.getTorqueCurrent());
    Serial.print(", CommandCurrent: ");
    Serial.print(command_current_1);
    Serial.print(" | ");

    Serial.print("ID: 2");
    Serial.print(", AccumDegree: ");
    Serial.print(motor2.getAccumPositionDeg());
    Serial.print(", RPS: ");
    Serial.print(motor2.getRps());
    Serial.print(", TorqueCurrent: ");
    Serial.print(motor2.getTorqueCurrent());
    Serial.print(", CommandCurrent: ");
    Serial.print(command_current_2);
    Serial.print(", Temp: ");
    Serial.print(motor2.getTemperature());
    Serial.println();
  }

  delay(1);
}
