// --- Configuration ---
// Set to 1 to enable anti-windup for the integral term, 0 to disable.
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
 * @brief A PID controller class with optional anti-windup.
 */
class PIDController {
public:
  /**
   * @brief Construct a new PIDController object.
   * @param kp Proportional gain.
   * @param ki Integral gain.
   * @param kd Derivative gain.
   * @param sample_freq Sampling frequency in Hz.
   * @param output_min Minimum output limit.
   * @param output_max Maximum output limit.
   */
  PIDController(float kp, float ki, float kd, float sample_freq, float output_min, float output_max)
    : kp_(kp),
      ki_(ki),
      kd_(kd),
      sample_freq_(sample_freq),
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
    float derivative = 0.0f;
    if (sample_freq_ > 0) {
      derivative = (error - prev_error_) * sample_freq_;
    }

    float p_term = kp_ * error;
    float d_term = kd_ * derivative;

#if USE_ANTI_WINDUP
    float i_term = ki_ * integral_;
    float output = p_term + i_term + d_term;
    float limited_output = std::clamp(output, output_min_, output_max_);

    // Conditional integration (anti-windup)
    if (output == limited_output) {
      integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    }
#else
    integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    float i_term = ki_ * integral_;
    float output = p_term + i_term + d_term;
    float limited_output = std::clamp(output, output_min_, output_max_);
#endif

    prev_error_ = error;
    return limited_output;
  }

  void reset() {
    integral_ = 0.0f;
    prev_error_ = 0.0f;
  }

private:
  float kp_, ki_, kd_;
  float sample_freq_;
  float output_min_, output_max_;
  float integral_;
  float prev_error_;
  float trapezoidal_multiplicand_;
};


// --- Global Objects and Parameters ---

robomaster::RoboMasterManager<1> manager(&CAN);
robomaster::C610 motor(manager, robomaster::C6x0Id::ID_1);

// --- Controllers ---
const float LOOP_FREQUENCY = 1000.0f;  // Hz

// Position Controller (Outer Loop) -> outputs target velocity
// Gains (Kp, Ki, Kd), Freq, Output Min (rad/s), Output Max (rad/s)
// These gains are examples and MUST be tuned.
const float POS_KP = 5.0f;
const float POS_KI = 0.0f;
const float POS_KD = 0.5f;
const float MAX_VELOCITY_RAD_S = 2000.0f;  // Velocity limit
PIDController position_controller(POS_KP, POS_KI, POS_KD, LOOP_FREQUENCY, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);

// Velocity Controller (Inner Loop) -> outputs current
// Gains (Kp, Ki, Kd), Freq, Output Min (mA), Output Max (mA)
// These gains are from previous tests and should be a good starting point.
const float VEL_KP = 5.0f;
const float VEL_KI = 68.5f;
const float VEL_KD = 0.0f;
const float MAX_CURRENT_MA = 10000.0f;  // Current limit
PIDController velocity_controller(VEL_KP, VEL_KI, VEL_KD, LOOP_FREQUENCY, -MAX_CURRENT_MA, MAX_CURRENT_MA);

// Target position
float target_position_rad = 0.0f;

void setup() {
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

  Serial.println("CAN bus started successfully!");
  Serial.println("--- C610 Cascade Position Control ---");
  Serial.println("Enter target position in radians (e.g., '1.57') and press Enter.");
  Serial.println("TargetPos_rad,CurrentPos_rad");
}

void loop() {
  unsigned long loop_start_us = micros();
  // Check for new target position from Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    float new_target_rad = input.toFloat();
    target_position_rad = new_target_rad;
    Serial.print("New target position set: ");
    Serial.print(new_target_rad, 4);
    Serial.println(" radians");
  }
  // --- Cascade Control ---
  manager.update();
  // Position Loop (Outer)
  float current_pos = motor.getAccumPositionRad();
  float target_vel = position_controller.compute(target_position_rad, current_pos);
  // Velocity Loop (Inner)
  float current_vel = motor.getVelocityRad();
  float output_current = velocity_controller.compute(target_vel, current_vel);
  // Set motor current and transmit
  motor.setCurrent(output_current);
  manager.transmit();
  // --- Serial Plotter Output ---
  static unsigned long last_print_us = 0;
  // 10ms interval: at 115200 baud a line takes >1ms to send, so printing
  // every 1ms would block Serial and break the 1kHz control loop timing.
  if (micros() - last_print_us > 10000) {
    Serial.print(target_position_rad, 4);
    Serial.print(",");
    Serial.println(current_pos, 4);
    last_print_us = micros();
  }

  // Maintain loop frequency
  while (micros() - loop_start_us < (1000000 / LOOP_FREQUENCY)) {
    // wait
  }
}
