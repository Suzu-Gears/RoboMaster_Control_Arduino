/*
  Compile_Time_Check

  This example demonstrates the compile-time conflict detection feature of the RoboMaster_Control library.
  It is designed to FAIL compilation to show how the static_assert mechanism works.

  The robomaster::Manager checks for two types of conflicts when you define your motors:
  1. Rx ID Conflict: Two motors cannot share the same feedback CAN ID.
  2. Tx Message Slot Conflict: Two motors cannot share the same CAN command message slot (a combination of CAN ID and buffer position).

  If a conflict is detected, the compiler will stop and show an error message pointing to a static_assert.
*/

// ▼▼▼ Select Board ▼▼▼ Uncomment only the board you are using
#define USE_BOARD_ARDUINO_R4
// #define USE_BOARD_PICO
// #define USE_BOARD_ESP32

#if defined(USE_BOARD_ARDUINO_R4)
#include <Arduino_CAN.h>
#elif defined(USE_BOARD_ESP32)
#include <ESP32_TWAI.h>
#elif defined(USE_BOARD_PICO)
#include <RP2040PIO_CAN.h>
#else
#error "Board not selected. Please enable one of USE_BOARD_... at the top of the file."
#endif

#include <RoboMaster_Control.h>  // Call after CAN library

//------------------------------------------------------------------
// PART 1: CONFLICTING CONFIGURATION
// The following setup WILL cause a compile-time error.
// Try compiling this sketch. You should see an error related to
// "Rx ID Conflict" or "Tx Message Slot Conflict".
//
// The conflict:
// - robomaster::C610::ID5 uses Rx ID 0x205 and Tx Slot (ID 0x1FF, index 0).
// - robomaster::GM6020::Voltage::ID1 uses Rx ID 0x205 and Tx Slot (ID 0x1FF, index 0).
// Both are identical, creating a conflict.
//------------------------------------------------------------------

namespace ConflictingAliases {
struct Motor1 {
  using spec = robomaster::C610::ID5;  // C610 with CAN ID 5
};
struct Motor2 {
  using spec = robomaster::GM6020::Voltage::ID1;  // GM6020 with dip switch set to ID 1
};
}  // namespace ConflictingAliases

// This manager definition will trigger the static_assert and fail to compile.
// Comment this out to proceed to PART 2.
robomaster::Manager<
  ConflictingAliases::Motor1,
  ConflictingAliases::Motor2>
  conflicting_manager(&CAN);


//------------------------------------------------------------------
// PART 2: VALID CONFIGURATION
// To fix the conflict, you must choose motors that have unique CAN IDs and command slots.
// For example, if we change the GM6020 to use ID 2, the conflict is resolved.
//
// The fix:
// - robomaster::C610::ID5 uses Rx ID 0x205 and Tx Slot (ID 0x1FF, index 0).
// - robomaster::GM6020::Voltage::ID2 uses Rx ID 0x206 and Tx Slot (ID 0x1FF, index 1).
// The IDs and slots are now unique.
//
// To test this, comment out the "conflicting_manager" above and uncomment the "valid_manager" below.
// The sketch will now compile successfully.
//------------------------------------------------------------------

/*
namespace ValidAliases {
struct Motor1 {
  using spec = robomaster::C610::ID5;
};
struct Motor2 {
  using spec = robomaster::GM6020::Voltage::ID2;
};
}  // namespace ValidAliases

robomaster::Manager<
  ValidAliases::Motor1,
  ValidAliases::Motor2>
  valid_manager(&CAN);
*/

void setup() {
  // This sketch does not have any runtime logic.
  // Its purpose is to demonstrate a compile-time check.
  Serial.begin(115200);
  Serial.println("This sketch is for demonstrating compile-time checks.");
  Serial.println("If you can see this message, you are using a valid (non-conflicting) motor configuration.");
}

void loop() {
  // Nothing to do here.
  delay(1000);
}
