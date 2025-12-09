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
const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

#else
#error "Board not selected. Please enable one of USE_BOARD_... at the top of the file."
#endif

#include <RoboMaster_Control.h>  // Call after CAN library

// 1. Define aliases for the motors you want to control using a macro.
// This is necessary for the Manager to identify each motor.
#define CREATE_MOTOR_ALIAS(AliasName, Spec) \
  struct AliasName { \
    using spec = Spec; \
  };

CREATE_MOTOR_ALIAS(MyC610_1, robomaster::C610::ID1);
CREATE_MOTOR_ALIAS(MyC620_2, robomaster::C620::ID2);
CREATE_MOTOR_ALIAS(MyGM6020_5, robomaster::GM6020::Voltage::ID5);

// 2. Create a single manager for all RoboMaster motors on the same CAN bus.
// Pass the aliases you defined as template arguments.
robomaster::Manager<
  MyC610_1,
  MyC620_2,
  MyGM6020_5>
  manager(&CAN);

// Helper to get motor objects from the manager
auto& motor_c610 = manager.get<MyC610_1>();
auto& motor_c620 = manager.get<MyC620_2>();
auto& motor_gm6020 = manager.get<MyGM6020_5>();

void setup() {
  Serial.begin(115200);

  // 3. Initialize the CAN bus.
#if defined(USE_BOARD_ARDUINO_R4)
  CAN.begin(CanBitRate::BR_1000k);
#elif defined(USE_BOARD_ESP32)
  CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);
#elif defined(USE_BOARD_PICO)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);
#endif
}

void loop() {
  // 4. Call manager.update() to receive data from all motors.
  manager.update();

  // 5. Implement control logic for each motor.
  const float kp = 100.0f;  // Proportional gain

  // C610 control (Current control)
  const float target_rps_1 = 10.0f;
  const float current_rps_1 = motor_c610.getRpm() / 60.0f;
  const float error_1 = target_rps_1 - current_rps_1;
  const float command_current_1 = kp * error_1;
  motor_c610.setCurrent(command_current_1);

  // C620 control (Current control)
  const float target_rps_2 = -10.0f;
  const float current_rps_2 = motor_c620.getRpm() / 60.0f;
  const float error_2 = target_rps_2 - current_rps_2;
  const float command_current_2 = kp * error_2;
  motor_c620.setCurrent(command_current_2);

  // GM6020 control (Voltage control)
  const float target_rps_3 = 5.0f;
  const float current_rps_3 = motor_gm6020.getRpm() / 60.0f;
  const float error_3 = target_rps_3 - current_rps_3;
  const float command_voltage_3 = kp * error_3;
  motor_gm6020.setVoltage(command_voltage_3);


  // 6. Call manager.transmit() to send commands to all motors.
  manager.transmit();

  // 7. Print motor data for debugging every 100ms.
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    last_print_time = millis();

    Serial.print("C610(1): ");
    Serial.print("RPM:");
    Serial.print(motor_c610.getRpm());
    Serial.print(", Temp:");
    Serial.print(motor_c610.getTemperature());
    Serial.print(" | ");

    Serial.print("C620(2): ");
    Serial.print("RPM:");
    Serial.print(motor_c620.getRpm());
    Serial.print(", Temp:");
    Serial.print(motor_c620.getTemperature());
    Serial.print(" | ");

    Serial.print("GM6020(5): ");
    Serial.print("RPM:");
    Serial.print(motor_gm6020.getRpm());
    Serial.print(", Temp:");
    Serial.print(motor_gm6020.getTemperature());
    Serial.println();
  }

  delay(1);
}
