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
#include <Arduino_CAN.h>
#endif

#include <RoboMaster_Control.h>

robomaster::RoboMasterManager manager(&CAN);
robomaster::C620 motor(manager, robomaster::C6x0Id::ID_1);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  bool can_ok = false;
#if defined(ARDUINO_ARCH_RENESAS)
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#elif defined(ARDUINO_ARCH_ESP32)
  can_ok = CAN.begin(CanBitRate::BR_1000k, CAN_TX_PIN, CAN_RX_PIN);
#elif defined(ARDUINO_ARCH_RP2040)
  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#else
  can_ok = CAN.begin(CanBitRate::BR_1000k);
#endif

  if (!can_ok) {
    Serial.println("CAN bus initialization failed!");
    while (1);
  }
  Serial.println("CAN bus initialized.");
}

void loop() {
  manager.update();

  static unsigned long last_print_time = 0;
  if (millis() - last_print_time > 100) {
    last_print_time = millis();

    Serial.print("Pos(raw): ");
    Serial.print(motor.getPositionRaw());
    Serial.print(", Pos(rad): ");
    Serial.print(motor.getPositionRad(), 2);
    Serial.print(", Pos(deg): ");
    Serial.print(motor.getPositionDeg(), 2);
    Serial.print(", AccPos(raw): ");
    Serial.print(motor.getAccumPosition());
    Serial.print(", AccPos(rad): ");
    Serial.print(motor.getAccumPositionRad());
    Serial.print(", AccPos(deg): ");
    Serial.print(motor.getAccumPositionDeg());
    Serial.print(", RPM: ");
    Serial.print(motor.getRpm());
    Serial.print(", RPS: ");
    Serial.print(motor.getRps());
    Serial.print(", Vel(rad/s): ");
    Serial.print(motor.getVelocityRad(), 2);
    Serial.print(", Current(mA): ");
    Serial.print(motor.getTorqueCurrent());
    Serial.print(", Temp(C): ");
    Serial.print(motor.getTemperature());
    Serial.print(", ErrorNum: ");
    Serial.print(motor.getErrorCode());
    robomaster::C6x0ErrorCode error_code = motor.getErrorCode();
    Serial.print(", Status: ");
    Serial.print(robomaster::getC6x0ErrorCodeString(error_code));

    Serial.println();
  }
  // motor.setCurrent(0);
  // manager.transmit();
  delay(1);
}
