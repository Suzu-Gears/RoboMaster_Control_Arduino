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
  PIDController(float kp, float ki, float kd, float sample_freq, float output_min, float output_max)
    : kp_(kp), ki_(ki), kd_(kd), sample_freq_(sample_freq), output_min_(output_min), output_max_(output_max), integral_(0.0f), prev_error_(0.0f) {
    if (sample_freq > 0) {
      trapezoidal_multiplicand_ = 1.0f / (sample_freq * 2.0f);
    } else {
      trapezoidal_multiplicand_ = 0.0f;
    }
  }

  float compute(float target, float current) {
    float error = target - current;
    float derivative = (sample_freq_ > 0) ? (error - prev_error_) * sample_freq_ : 0.0f;
#if USE_ANTI_WINDUP
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    float limited_output = std::clamp(output, output_min_, output_max_);
    if (output == limited_output) {
      integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    }
#else
    integral_ += (error + prev_error_) * trapezoidal_multiplicand_;
    float output = kp_ * error + ki_ * integral_ + kd_ * derivative;
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
  float kp_, ki_, kd_, sample_freq_, output_min_, output_max_, integral_, prev_error_, trapezoidal_multiplicand_;
};

// --- Global Objects and Parameters ---

robomaster::RoboMasterManager<1> manager(&CAN);
robomaster::C610 motor(manager, robomaster::C6x0Id::ID_1);

// --- Mechanical Parameters ---
const float GEAR_RATIO = 36.0f;

// --- Controllers ---
const float LOOP_FREQUENCY = 1000.0f;  // Hz
const float POS_KP = 10.0f;
const float POS_KI = 0.0f;
const float POS_KD = 2.0f;
const float MAX_VELOCITY_RAD_S = 2000.0f;
PIDController position_controller(POS_KP, POS_KI, POS_KD, LOOP_FREQUENCY, -MAX_VELOCITY_RAD_S, MAX_VELOCITY_RAD_S);

const float VEL_KP = 5.0f;
const float VEL_KI = 68.5f;
const float VEL_KD = 0.0f;
const float MAX_CURRENT_MA = 10000.0f;  // Current limit
PIDController velocity_controller(VEL_KP, VEL_KI, VEL_KD, LOOP_FREQUENCY, -MAX_CURRENT_MA, MAX_CURRENT_MA);

// --- Target State ---
float output_shaft_target_rad = 0.0f;
unsigned long last_step_ms = 0;
const unsigned long STEP_INTERVAL_MS = 1500;

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
    Serial.println("FATAL: CAN bus initialization failed!");
    while (1);
  }

  Serial.println("CAN bus started successfully!");
  Serial.println("--- C610 Position Control with Gearbox ---");
  Serial.println("Output shaft will move 90 degrees every second.");
  Serial.println("TargetPos_rad,CurrentPos_rad");
  last_step_ms = millis();
}

void loop() {
  unsigned long loop_start_us = micros();

  // --- Step Target Update ---
  if (millis() - last_step_ms >= STEP_INTERVAL_MS) {
    last_step_ms += STEP_INTERVAL_MS;
    output_shaft_target_rad += 2.0f * PI;
  }

  // --- Cascade Control ---
  manager.update();

  // Convert output shaft target to motor shaft target
  float motor_shaft_target_rad = output_shaft_target_rad * GEAR_RATIO;

  // Position Loop (Outer) - Operates on motor shaft values
  float motor_shaft_current_pos = motor.getAccumPositionRad();
  float motor_shaft_target_vel = position_controller.compute(motor_shaft_target_rad, motor_shaft_current_pos);

  // Velocity Loop (Inner) - Operates on motor shaft values
  float motor_shaft_current_vel = motor.getVelocityRad();
  float output_current = velocity_controller.compute(motor_shaft_target_vel, motor_shaft_current_vel);

  motor.setCurrent(output_current);
  manager.transmit();

  // --- Serial Plotter Output ---
  static unsigned long last_print_us = 0;
  // 10ms interval: at 115200 baud a line takes >1ms to send, so printing
  // every 1ms would block Serial and break the 1kHz control loop timing.
  if (micros() - last_print_us > 10000) {
    float output_shaft_current_pos = motor_shaft_current_pos / GEAR_RATIO;
    Serial.print(output_shaft_target_rad, 4);
    Serial.print(",");
    Serial.println(output_shaft_current_pos, 4);
    last_print_us = micros();
  }

  while (micros() - loop_start_us < (1000000 / LOOP_FREQUENCY));
}
