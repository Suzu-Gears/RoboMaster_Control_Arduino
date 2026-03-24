#include <RP2040PIO_CAN.h>
#include <NeoPixelConnect.h>
#include <RoboMaster_Control.h>
#include "pid_controller.hpp"
#include "PS3.h"
#include <cmath>  // For PI constant

PidController motor_pid;
PS3 ps3;

// Pin Definitions
const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;
const uint32_t SBDBT_TX_PIN = 12;
const uint32_t SBDBT_RX_PIN = 13;
const uint32_t NEOPIXEL_PIN = 15;

// PID Constants
const float KP = 50.0f;
// Set KI to 0 to disable integral action for safety, preventing runaway.
const float KI = 5.0f;

// Physical Constants
const float GEAR_RATIO = 36.0f;
const float WHEEL_CIRCUMFERENCE_MM = 94.2477f;  // 仮定値

// Speed Levels
const float SPEED_LEVELS[] = { 95.5f, 252.5f, 353.5f, 505.0f };
const int NUM_SPEED_LEVELS = sizeof(SPEED_LEVELS) / sizeof(SPEED_LEVELS[0]);
int current_speed_level_index = 0;

// Control Modes
enum ControlMode {
  MODE_JOYSTICK,
  MODE_DISTANCE_MOVE
};
ControlMode current_mode = MODE_JOYSTICK;

// Distance Move Variables
float distance_target_mm = 0.0f;
float distance_start_pos_mm = 0.0f;
float total_distance_mm = 0.0f;
// unsigned long last_time_distance_calc = 0; // Not needed with getAccumPositionRad()

// Motor Current Limit (in mA)
// Max current for C610 is around 10000mA (10A). Setting a safer limit.
const float MAX_CURRENT_MA = 2000.0f;

using namespace robomaster;

RoboMasterManager manager(&CAN);
C610 motor_c610(manager, C6x0Id::ID_3);

// Use State Machine 1 on pio1 to avoid potential conflicts
NeoPixelConnect neop(NEOPIXEL_PIN, 1, pio1, 1);

// Colors are represented as 0xRRGGBB
const uint32_t SPEED_LEVEL_COLORS[] = {
  0x0000FF,  // Level 0: Blue
  0x00FF00,  // Level 1: Green
  0xFFFF00,  // Level 2: Yellow
  0xFF0000   // Level 3: Red
};

void setup() {
  Serial.begin(115200);
  Serial1.setRX(SBDBT_RX_PIN);
  Serial1.setTX(SBDBT_TX_PIN);
  Serial1.begin(115200);
  ps3.setSerial(&Serial1);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  motor_pid.init({ 0.001f, KP, KI, 0.0f, 0.0f, MAX_CURRENT_MA, 10.0f });

  neop.neoPixelSetValue(0, 0, 0, 0, true);
  // last_time_distance_calc = millis(); // Not needed
}

void loop() {
  // --- Updates ---
  manager.update();
  ps3.update();

  // --- Distance Calculation ---
  // Calculate total_distance_mm directly from accumulated motor position
  // motor_c610.getAccumPositionRad() is motor shaft rotation, so divide by GEAR_RATIO for wheel rotation
  total_distance_mm = (motor_c610.getAccumPositionRad() / (2.0f * PI * GEAR_RATIO));

  // current_velocity_mm_s is still needed for PID feedback.
  // It's derived from motor_c610.getRps(), which is instantaneous rotational speed, not cumulative position.
  float current_velocity_mm_s = (motor_c610.getRps() / GEAR_RATIO) * WHEEL_CIRCUMFERENCE_MM;

  // --- Controller Input & Mode Selection ---
  float rawLeftX = ps3.getAxis(PS3Axis::LEFT_X);

  // Speed level selection with D-pad up/down
  if (ps3.getKeyDown(PS3Key::UP)) {
    if (current_speed_level_index < NUM_SPEED_LEVELS - 1) {
      current_speed_level_index++;
    }
  }
  if (ps3.getKeyDown(PS3Key::DOWN)) {
    if (current_speed_level_index > 0) {
      current_speed_level_index--;
    }
  }

  // If joystick is moved significantly, switch to joystick mode
  if (abs(rawLeftX) > 0.1f) {
    current_mode = MODE_JOYSTICK;
  }

  // If D-pad left/right is pressed, switch to distance move mode
  if (ps3.getKeyDown(PS3Key::RIGHT)) {
    current_mode = MODE_DISTANCE_MOVE;
    distance_target_mm = 1.0f;
    distance_start_pos_mm = total_distance_mm;
  }
  if (ps3.getKeyDown(PS3Key::LEFT)) {
    current_mode = MODE_DISTANCE_MOVE;
    distance_target_mm = -1.0f;
    distance_start_pos_mm = total_distance_mm;
  }

  // --- Target Velocity Calculation ---
  float target_velocity = 0.0f;
  switch (current_mode) {
    case MODE_JOYSTICK:
      // Limit joystick speed to 50mm/s
      target_velocity = 50.0f * rawLeftX;
      break;

    case MODE_DISTANCE_MOVE:
      float distance_traveled = total_distance_mm - distance_start_pos_mm;
      if (abs(distance_traveled) < abs(distance_target_mm)) {
        // Not reached the target yet, keep moving at the selected speed level
        float direction = (distance_target_mm > 0) ? 1.0f : -1.0f;
        target_velocity = SPEED_LEVELS[current_speed_level_index] * direction;
      } else {
        // Reached the target, stop and switch back to joystick mode
        target_velocity = 0.0f;
        current_mode = MODE_JOYSTICK;
      }
      break;
  }

  // Invert the final target velocity to fix the direction issue globally.
  target_velocity = -target_velocity;

  // --- Neopixel Update ---
  // Set color by unpacking the 32-bit color value into R, G, B components
  uint32_t color = SPEED_LEVEL_COLORS[current_speed_level_index];
  uint8_t r = (color >> 16) & 0xFF;
  uint8_t g = (color >> 8) & 0xFF;
  uint8_t b = color & 0xFF;
  neop.neoPixelSetValue(0, r, g, b, true);

  // --- Motor Control ---
  float output_current = motor_pid.solve(target_velocity - current_velocity_mm_s);

  motor_c610.setCurrent(output_current);
  manager.transmit();

  // --- Serial Output for Debugging ---
  Serial.print(current_mode == MODE_JOYSTICK ? "JOY" : "DIST");
  Serial.print(", LV:");
  Serial.print(current_speed_level_index);
  Serial.print(", T_Vel:");
  Serial.print(target_velocity);
  Serial.print(", C_Vel:");
  Serial.print(current_velocity_mm_s);
  Serial.print(", Dist:");
  Serial.print(total_distance_mm);
  Serial.print(", Current:");  // Debug output for current
  Serial.print(output_current);
  Serial.println();

  delay(1);
}
