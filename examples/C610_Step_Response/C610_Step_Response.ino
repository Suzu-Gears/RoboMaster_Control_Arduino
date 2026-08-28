// Board-specific CAN settings are handled automatically below
#if defined(ARDUINO_ARCH_RENESAS)
#include <Arduino_CAN.h>  // For Arduino UNO R4, etc.

#elif defined(ARDUINO_ARCH_ESP32)
#include <ESP32_TWAI.h>  // For ESP32 series
const gpio_num_t CAN_TX_PIN = GPIO_NUM_21;
const gpio_num_t CAN_RX_PIN = GPIO_NUM_22;

#elif defined(ARDUINO_ARCH_RP2040)
#include <RP2040PIO_CAN.h>  // For RP2040, RP2350, etc.
const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;

#else
#warning "This board is not officially supported. Please include your CAN library and define CAN pins if necessary before including RoboMaster.h"
#include <Arduino_CAN.h>
#endif

#include <RoboMaster_Control.h>

robomaster::RoboMasterManager<1> manager(&CAN);
robomaster::C620 motor(manager, robomaster::C6x0Id::ID_5);

const int NUM_SAMPLES = 1000;
const int SAMPLE_INTERVAL_MS = 1;
const float STEP_CURRENT_MA = 20000.0f;

void setup() {
  static float feedback_current_log[NUM_SAMPLES];
  static int16_t rpm_log[NUM_SAMPLES];

  Serial.begin(115200);
  while (!Serial && millis() < 3000) {
    ;  // wait for serial port to connect.
  }
  delay(1000);  // Give some time for Serial monitor to open

  // --- CAN Bus Initialization ---
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
    Serial.println("FATAL: CAN bus initialization failed!");
    while (1);
  }

  Serial.println("CAN bus started successfully!");
  Serial.println("--- Velocity Step Response Test ---");
  Serial.print("Applying step current of ");
  Serial.print(STEP_CURRENT_MA);
  Serial.println(" mA for 1 second...");

  // --- Data Logging Phase ---
  for (int i = 0; i < NUM_SAMPLES; i++) {
    unsigned long start_time = micros();

    manager.update();
    motor.setCurrent(STEP_CURRENT_MA);
    manager.transmit();

    feedback_current_log[i] = motor.getTorqueCurrent();
    rpm_log[i] = motor.getRpm();

    while (micros() - start_time < (unsigned long)SAMPLE_INTERVAL_MS * 1000) {
      // busy wait for accurate timing
    }
  }

  // Stop the motor
  motor.setCurrent(0);
  manager.transmit();
  delay(10);  // Ensure the stop command is sent
  motor.setCurrent(0);
  manager.transmit();

  // --- Data Output Phase ---
  Serial.println("\n--- Data Logging Complete ---");
  Serial.println("Time_ms,TargetCurrent_mA,FeedbackCurrent_mA,RPM");

  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print(i * SAMPLE_INTERVAL_MS);
    Serial.print(",");
    Serial.print(feedback_current_log[i], 0);
    Serial.print(",");
    Serial.println(rpm_log[i]);
  }

  Serial.println("\n--- Test Finished. ---");
}

void loop() {
  // Everything is done in setup(), so loop is empty.
}
