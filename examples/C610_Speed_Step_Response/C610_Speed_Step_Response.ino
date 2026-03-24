// --- Configuration ---
// Set to 1 to enable anti-windup, 0 to disable.
#define USE_ANTI_WINDUP 1

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
#include <Arduino_CAN.h>
#endif

#include <RoboMaster_Control.h>

/**
 * @brief A PI controller class with optional anti-windup.
 */
class PIController {
public:
  PIController(float kp, float ki, float sample_freq, float output_min, float output_max)
    : kp_(kp),
      ki_(ki),
      output_min_(output_min),
      output_max_(output_max),
      integral_(0.0f),
      prev_error_(0.0f) {
    if (sample_freq > 0) {
      trapezoidal_multiplicand_ = 1.0f / (sample_freq * 2.0f);
    } else {
      trapezoidal_multiplicand_ = 0.0f;
    }
  }

  float compute(float target, float current) {
    float error = target - current;
#if USE_ANTI_WINDUP
    float output = kp_ * error + ki_ * integral_;
    float limited_output = std::clamp(output, output_min_, output_max_);
    if (output == limited_output) {
      integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    }
    prev_error_ = error;
    return limited_output;
#else
    integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    prev_error_ = error;
    float output = kp_ * error + ki_ * integral_;
    return std::clamp(output, output_min_, output_max_);
#endif
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
  }

private:
  float kp_, ki_;
  float output_min_, output_max_;
  float integral_;
  float prev_error_;
  float trapezoidal_multiplicand_;
};

// --- Global Objects and Parameters ---
robomaster::RoboMasterManager<1> manager(&CAN);
robomaster::C610 motor(manager, robomaster::C6x0Id::ID_1);

// Test parameters
const int NUM_SAMPLES = 1000;
const int SAMPLE_INTERVAL_MS = 1;
const float LOOP_FREQUENCY = 1000.0f / SAMPLE_INTERVAL_MS;
const float STEP_SPEED_RAD_S = 2000.0f;  // Target speed for the step response

// PI Controller for velocity
const float VELOCITY_KP = 5.0f;
const float VELOCITY_KI = 68.5f;

void setup() {
  // --- Data Logging Arrays ---
  // Use static to avoid stack overflow on some microcontrollers
  static float target_speed_log[NUM_SAMPLES];
  static float current_speed_log[NUM_SAMPLES];
  static float current_position_log[NUM_SAMPLES];
  static float output_current_log[NUM_SAMPLES];
  static float feedback_current_log[NUM_SAMPLES];

  Serial.begin(115200);
  while (!Serial && millis() < 3000);

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

  // --- Controller Initialization ---
  PIController speed_controller(VELOCITY_KP, VELOCITY_KI, LOOP_FREQUENCY, -10000.0f, 10000.0f);
  // Reset motor position at the beginning
  manager.update();

  Serial.println("CAN bus started successfully!");
  Serial.println("--- Speed Control Step Response Test ---");
  Serial.print("Applying step speed of ");
  Serial.print(STEP_SPEED_RAD_S);
  Serial.println(" rad/s for 1 second...");
  delay(1000);  // Give user time to open plotter

  // --- Data Logging Phase ---
  for (int i = 0; i < NUM_SAMPLES; i++) {
    unsigned long start_time_us = micros();

    manager.update();

    float current_speed = motor.getVelocityRad();
    float output_current = speed_controller.compute(STEP_SPEED_RAD_S, current_speed);
    motor.setCurrent(output_current);

    manager.transmit();

    // Log data
    target_speed_log[i] = STEP_SPEED_RAD_S;
    current_speed_log[i] = current_speed;
    current_position_log[i] = motor.getAccumPositionRad();
    output_current_log[i] = output_current;
    feedback_current_log[i] = motor.getTorqueCurrent();

    while (micros() - start_time_us < (unsigned long)SAMPLE_INTERVAL_MS * 1000) {
      // busy wait for accurate timing
    }
  }

  // Stop the motor
  motor.setCurrent(0);
  manager.transmit();
  delay(10);
  manager.transmit();

  // --- Data Output Phase ---
  Serial.println("\n--- Data Logging Complete ---");
  Serial.println("Time_ms,TargetSpeed_rad/s,CurrentSpeed_rad/s,CurrentPosition_rad,OutputCurrent_mA,FeedbackCurrent_mA");

  for (int i = 0; i < NUM_SAMPLES; i++) {
    Serial.print(i * SAMPLE_INTERVAL_MS);
    Serial.print(",");
    Serial.print(target_speed_log[i], 4);
    Serial.print(",");
    Serial.print(current_speed_log[i], 4);
    Serial.print(",");
    Serial.print(current_position_log[i], 4);
    Serial.print(",");
    Serial.print(output_current_log[i], 0);
    Serial.print(",");
    Serial.print(feedback_current_log[i], 0);
    Serial.println();
  }

  Serial.println("\n--- Test Finished. ---");
}

void loop() {
  // Everything is done in setup(), so loop is empty.
}
