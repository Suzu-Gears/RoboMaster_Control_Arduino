// Board-specific CAN settings are handled automatically below
#if defined(ARDUINO_ARCH_RENESAS)
#include <Arduino_CAN.h>  // For Arduino UNO R4, etc.

#elif defined(ARDUINO_ARCH_ESP32)
#include <ESP32_TWAI.h>  // For ESP32 series
const gpio_num_t CAN_TX_PIN = 22;
const gpio_num_t CAN_RX_PIN = 21;

#elif defined(ARDUINO_ARCH_RP2040)
#include <RP2040PIO_CAN.h>  // For RP2040, RP2350, etc.
const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

#else
#warning "This board is not officially supported. Please include your CAN library and define CAN pins if necessary before including RoboMaster.h"
// As a fallback, try to include Arduino_CAN.h
#include <Arduino_CAN.h>
#endif

#include <RoboMaster_Control.h>

using namespace robomaster;

// 1. Create a single manager for all RoboMaster motors on the same CAN bus.
// You can specify the maximum number of motors in the template argument
// to save memory. If omitted, the default is 15.
// e.g. RoboMasterManager<3> manager(&CAN);
RoboMasterManager manager(&CAN);

// 2. Create motor objects using their dedicated classes.
C610 motor_c610(manager, C6x0Id::ID_1);
C620 motor_c620(manager, C6x0Id::ID_5);
GM6020_Voltage motor_gm6020(manager, GM6020Id::ID_5);

// --- Examples of ID Conflicts (uncomment to test) ---
// The program will halt in setup() if a conflict is detected.

// Example 1: Same motor type, same ID (causes both Rx and Tx conflict)
// C610 motor_c610_conflict(manager, C6x0Id::ID_1);

// Example 2: Different motor types, but same Rx ID
// C620's ID 5 and GM6020's ID 1 both use Rx ID 0x205.
// C620 motor_c620_rx_conflict(manager, C6x0Id::ID_5);
// GM6020_Voltage motor_gm6020_rx_conflict(manager, GM6020Id::ID_1);

// Example 3: Different motor types, but same Tx ID and buffer slot
// C620's ID 5 and GM6020_Voltage's ID 1 both use Tx message 0x1FF, slot 0.
// C620 motor_c620_tx_conflict(manager, C6x0Id::ID_5);
// GM6020_Voltage motor_gm6020_tx_conflict(manager, GM6020Id::ID_1);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);  // Wait for Serial to initialize (max 3 seconds)

  // Print details for each successfully registered motor.
  Serial.println("--- Motor Initialization Details ---");
  for (size_t i = 0; i < manager.getMotorCount(); ++i) {
    const auto& config = manager.getMotorConfig(i);
    Serial.print("Motor ");
    Serial.print(i);
    Serial.print(": Type=");
    Serial.print(getMotorTypeString(config.motor_type));
    Serial.print(", RxID=0x");
    Serial.print(config.rx_id, HEX);
    Serial.print(", TxID=0x");
    Serial.print(config.tx_id, HEX);
    Serial.print(", TxIdx=");
    Serial.println(config.tx_buf_idx);
  }
  Serial.println("------------------------------------");

  // 3. Check for motor ID conflicts after defining all motors.
  if (manager.hasConflict()) {
    auto info = manager.getConflictInfo().value();
    Serial.print("FATAL: Motor ID conflict detected on model ");
    Serial.print(getMotorTypeString(info.motor_type));
    Serial.print(" with ID ");
    Serial.println(info.id);
    while (1);
  }

  Serial.println("--- Motor initialization successful ---");

  // 4. Initialize the CAN bus.
  bool can_ok = false;
#if defined(ARDUINO_ARCH_RENESAS)
  can_ok = CAN.begin(CanBitRate::BR_1000k);

#elif defined(ARDUINO_ARCH_ESP32)
  can_ok = CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);

#elif defined(ARDUINO_ARCH_RP2040)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  can_ok = CAN.begin(CanBitRate::BR_1000k);

#else  // Fallback for other boards
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#endif
  if (!can_ok) {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
}

void loop() {
  // 5. Call manager.update() to receive data from all motors.
  manager.update();

  // 6. Implement control logic for each motor.
  const float kp = 100.0f;  // Proportional gain

  // C610 control (Current control)
  const float target_rps_1 = 10.0f;
  const float current_rps_1 = motor_c610.getRps();
  const float error_1 = target_rps_1 - current_rps_1;
  const float command_current_1 = kp * error_1;
  motor_c610.setCurrent(command_current_1);

  // C620 control (Current control)
  const float target_rps_2 = -10.0f;
  const float current_rps_2 = motor_c620.getRps();
  const float error_2 = target_rps_2 - current_rps_2;
  const float command_current_2 = kp * error_2;
  motor_c620.setCurrent(command_current_2);

  // GM6020 control (Voltage control)
  const float target_rps_3 = 5.0f;
  const float current_rps_3 = motor_gm6020.getRps();
  const float error_3 = target_rps_3 - current_rps_3;
  const float command_voltage_3 = kp * error_3;
  motor_gm6020.setVoltage(command_voltage_3);

  // 7. Call manager.transmit() to send commands to all motors.
  manager.transmit();

  // 8. Print all motor data.
  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    last_print_time = millis();
    Serial.print("C610(1): RPM: ");
    Serial.print(motor_c610.getRpm());
    Serial.print("rpm, ");
    Serial.print("Current: ");
    Serial.print(motor_c610.getTorqueCurrent());
    Serial.print("mA, Err:");
    Serial.print(motor_c610.getErrorCode());
    Serial.print(" : ");
    Serial.print(getC6x0ErrorCodeString(motor_c610.getErrorCode()));
    Serial.print(" | ");

    Serial.print("C620(2): RPM: ");
    Serial.print(motor_c620.getRpm());
    Serial.print("rpm, ");
    Serial.print("Current: ");
    Serial.print(motor_c620.getTorqueCurrent());
    Serial.print("mA, Temp:");
    Serial.print(motor_c620.getTemperature());
    Serial.print("℃, Err:");
    Serial.print(motor_c620.getErrorCode());
    Serial.print(" : ");
    Serial.print(getC6x0ErrorCodeString(motor_c620.getErrorCode()));
    Serial.print(" | ");

    Serial.print("GM6020(5): RPM: ");
    Serial.print(motor_gm6020.getRpm());
    Serial.print("rpm, ");
    Serial.print("Current: ");
    Serial.print(motor_gm6020.getTorqueCurrent());
    Serial.print("mA, Temp:");
    Serial.print(motor_gm6020.getTemperature());
    Serial.print("℃");
    Serial.println();
  }

  delay(1);
}
