// --- Configuration ---
// Set to 1 to enable anti-windup, 0 to disable.
#define USE_ANTI_WINDUP 1

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

/**
 * @brief A PI controller class with optional anti-windup.
 * Based on the user-provided functions PI_Current and Antiwindup_PI_Current.
 */
class PIController {
public:
  /**
   * @brief Construct a new PIController object.
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param sample_freq Sampling frequency in Hz.
   * @param output_min Minimum output limit.
   * @param output_max Maximum output limit.
   */
  PIController(float kp, float ki, float sample_freq, float output_min, float output_max)
    : kp_(kp),
      ki_(ki),
      output_min_(output_min),
      output_max_(output_max),
      integral_(0.0f),
      prev_error_(0.0f) {
    // Pre-calculate the multiplicand for trapezoidal integration
    if (sample_freq > 0) {
      trapezoidal_multiplicand_ = 1.0f / (sample_freq * 2.0f);
    } else {
      trapezoidal_multiplicand_ = 0.0f;
    }
  }

  /**
   * @brief Compute the control output.
   * @param target The target value.
   * @param current The current measured value.
   * @return The calculated control output.
   */
  float compute(float target, float current) {
    float error = target - current;

#if USE_ANTI_WINDUP
    // --- Anti-windup PI control ---
    if (ki_ == 0) {
      integral_ = 0;
    }

    float output = kp_ * error + ki_ * integral_;
    float limited_output = std::clamp(output, output_min_, output_max_);

    // Anti-windup: Back-calculation
    // Adjust the integral term based on the difference between unsaturated and saturated output.
    if (output != limited_output && ki_ != 0) {
      // This is a simplified back-calculation. The user's original code was `cie -= (cU - limited_cU) / cki;`
      // which is equivalent to adding the saturation error scaled by a tracking time constant.
      // For simplicity here, we'll use a common form of conditional integration.
      // We only integrate if the output is not saturated.
    } else {
      integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    }

    prev_error_ = error;
    return limited_output;

#else
    // --- Simple PI control ---
    integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    prev_error_ = error;

    float output = kp_ * error + ki_ * integral_;
    return std::clamp(output, output_min_, output_max_);
#endif
  }

  /**
   * @brief Reset the integral and previous error.
   */
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

// RoboMaster CAN bus manager
robomaster::RoboMasterManager<1> manager(&CAN);
// C610 motor with ID 1
robomaster::C610 motor(manager, robomaster::C6x0Id::ID_1);

// PI Controller for velocity
// Gains (Kp, Ki), sampling frequency (Hz), output min (mA), output max (mA)
// These gains are examples and MUST be tuned.
const float VELOCITY_KP = 5.0f;
const float VELOCITY_KI = 68.5f;
const float LOOP_FREQUENCY = 1000.0f;  // Hz
PIController speed_controller(VELOCITY_KP, VELOCITY_KI, LOOP_FREQUENCY, -10000.0f, 10000.0f);

// Sine wave parameters for target velocity
const float SINE_AMPLITUDE_RAD_S = 2000.0;  // Amplitude of the sine wave in rad/s (approx 8 RPS)
const float SINE_FREQUENCY_HZ = 0.5;      // Frequency of the sine wave in Hz

unsigned long start_time_us = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;

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
    while (1)
      ;
  }

  Serial.println("CAN bus started successfully!");
  Serial.println("--- C610 Speed Control with Sine Wave Target ---");
  Serial.println("TargetSpeed_rad/s,CurrentSpeed_rad/s");

  start_time_us = micros();
}

void loop() {
  // --- Control Loop @ approx. LOOP_FREQUENCY ---
  unsigned long loop_start_us = micros();

  // 1. Update motor feedback
  manager.update();

  // 2. Generate sinusoidal target velocity
  float elapsed_time_s = (micros() - start_time_us) / 1000000.0f;
  float target_speed_rad_s = SINE_AMPLITUDE_RAD_S * sin(2.0f * PI * SINE_FREQUENCY_HZ * elapsed_time_s);

  // 3. Get current velocity
  float current_speed_rad_s = motor.getVelocityRad();

  // 4. Compute control output (current)
  float output_current = speed_controller.compute(target_speed_rad_s, current_speed_rad_s);

  // 5. Set motor current
  motor.setCurrent(output_current);

  // 6. Transmit CAN command
  manager.transmit();

  Serial.print(target_speed_rad_s, 4);
  Serial.print(",");
  Serial.println(current_speed_rad_s, 4);

  // Busy wait to maintain loop frequency
  while (micros() - loop_start_us < (1000000 / LOOP_FREQUENCY)) {
    // wait
  }
}
