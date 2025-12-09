// This code is based on https://github.com/tutrc-freshman/TUTRC_ArduinoLib/blob/master/src/C6x0.h and https://github.com/eyr1n/halx_driver/blob/master/include/halx/driver/c6x0.hpp

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#if !__has_include(<api/HardwareCAN.h>)
#error "C6x0.h: include a HardwareCAN provider (e.g. <Arduino_CAN.h>, <ESP32_TWAI.h>, <RP2040PIO_CAN.h>) before this file."
#endif

#include <api/HardwareCAN.h>

enum class C6x0Type {
  C610,
  C620,
};

enum class C6x0Id {
  ID_1,
  ID_2,
  ID_3,
  ID_4,
  ID_5,
  ID_6,
  ID_7,
  ID_8,
};

// If multiple warnings or errors occur at the same time, the error code with higher priority (smaller value) will be reported first.
enum class C6x0ErrorCode : uint8_t {
  NO_ERROR = 0,                         // C610, C620 | No abnormality (normal)
  MOTOR_CHIP_ACCESS_FAILURE = 1,        //       C620 | Cannot access motor memory chip (detected during power-on self-test)
  MSC_SUPPLY_OVER_VOLTAGE = 2,          // C610, C620 | MSC supply voltage too high (detected during power-on self-test)
  THREE_PHASE_CABLE_NOT_CONNECTED = 3,  // C610, C620 | Three-phase cable to motor not connected
  POSITION_SENSOR_SIGNAL_LOST = 4,      // C610, C620 | Signal lost on 4-pin position sensor cable connected to motor
  MOTOR_TEMPERATURE_CRITICAL = 5,       //       C620 | Motor temperature critical (e.g., >=180°C)
  MOTOR_STALLED = 6,                    // C610       | Motor stalled
  MOTOR_CALIBRATION_FAILED = 7,         // C610, C620 | Motor calibration failed
  MOTOR_OVER_TEMPERATURE = 8,           //       C620 | Motor over temperature (e.g., >=125°C)
  NO_CAN_MESSAGE = 99,                  // C610, C620 | "Library defined" No CAN message has been received yet
};

class C6x0Manager {
public:
  C6x0Manager(arduino::HardwareCAN *can) : can_{ can } {}

  void update() {
    while (can_->available()) {
      CanMsg msg = can_->read();
      uint32_t id = msg.getStandardId();
      if (0x201 <= id && id < 0x201 + 8) {
        Params &motor = params_[id - 0x201];
        int16_t position = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
        if (motor.prev_position_) {
          const int16_t delta = position - *motor.prev_position_;
          motor.position_ += ((delta + 4096) & 8191) - 4096;
        } else {
          motor.position_ = position;
        }
        motor.prev_position_ = position;
        motor.rpm_ = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
        motor.current_raw_ = static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
        motor.temperature = static_cast<int8_t>(msg.data[6]);
        motor.error_code = static_cast<C6x0ErrorCode>(msg.data[7]);
      }
    }
  }

  bool transmit() {
    if (!transmit_current(0x200, 0)) {
      return false;
    }
    if (!transmit_current(0x1FF, 4)) {
      return false;
    }
    return true;
  }

  int16_t getPosition(C6x0Id id) const {
    return params_[toUnderlying(id)].prev_position_.value_or(0);
  }

  int64_t getAccumPosition(C6x0Id id) const {
    return params_[toUnderlying(id)].position_;
  }

  int16_t getRpm(C6x0Id id) const {
    return params_[toUnderlying(id)].rpm_;
  }

  int16_t getCurrentRaw(C6x0Id id) const {
    return params_[toUnderlying(id)].current_raw_;
  }

  int8_t getTemperature(C6x0Id id) const {
    return params_[toUnderlying(id)].temperature;
  }

  C6x0ErrorCode getErrorCode(C6x0Id id) const {
    return params_[toUnderlying(id)].error_code;
  }

  void setCurrentRefRaw(C6x0Id id, int16_t current) {
    params_[toUnderlying(id)].current_ref_ = current;
  }

private:
  bool transmit_current(uint32_t can_id, size_t start_index) {
    if (std::any_of(params_.begin() + start_index, params_.begin() + start_index + 4, [](const Params &p) {
          return p.current_ref_.has_value();
        })) {
      CanMsg msg;
      msg.id = CanStandardId(can_id);
      msg.data_length = 8;
      for (size_t i = 0; i < 4; ++i) {
        const int16_t current_ref = params_[i + start_index].current_ref_.value_or(0);
        msg.data[i * 2] = current_ref >> 8;
        msg.data[i * 2 + 1] = current_ref;
      }
      return can_->write(msg) >= 0;
    }
    return true;
  }

  struct Params {
    int64_t position_ = 0;
    std::optional<int16_t> prev_position_;
    int16_t rpm_ = 0;
    int16_t current_raw_ = 0;
    std::optional<int16_t> current_ref_;
    int8_t temperature = 0;
    C6x0ErrorCode error_code = C6x0ErrorCode::NO_CAN_MESSAGE;
  };

  arduino::HardwareCAN *can_;
  std::array<Params, 8> params_{};

  template<class T>
  constexpr typename std::underlying_type<T>::type
  toUnderlying(T value) const noexcept {
    return static_cast<typename std::underlying_type<T>::type>(value);
  }
};

namespace detail {
class C6x0Base {
public:
  C6x0Base(C6x0Manager &manager, C6x0Id id)
    : manager_{ manager }, id_{ id } {}

  ~C6x0Base() {
    manager_.setCurrentRefRaw(id_, 0);
  }

  C6x0Base(const C6x0Base &) = delete;
  C6x0Base &operator=(const C6x0Base &) = delete;

  // Angle range: 0 to 8191 (0° to 360°)
  int16_t getPosition() const {
    return manager_.getPosition(id_);
  }

  // Angle range: 0 to 2PI [rad]
  float getPositionRad() const {
    return static_cast<float>(getPosition()) / 8192.0f * 2.0f * M_PI;
  }

  // Angle range: 0 to 360 [deg]
  float getPositionDeg() const {
    return static_cast<float>(getPosition()) / 8192.0f * 360.0f;
  }

  // Unit is encoder ticks (8192 ticks per revolution)
  int64_t getAccumPosition() const {
    return manager_.getAccumPosition(id_);
  }

  float getAccumPositionRad() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 2.0f * M_PI;
  }

  float getAccumPositionDeg() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 360.0f;
  }

  int16_t getRpm() const {
    return manager_.getRpm(id_);
  }

  float getRps() const {
    return static_cast<float>(getRpm()) / 60.0f;
  }

  // [rad/s]
  float getVelocityRad() const {
    return static_cast<float>(getRpm()) / 60.0f * 2.0f * M_PI;
  }

  // [°C]
  int8_t getTemperature() const {
    return manager_.getTemperature(id_);
  }

  // C610 C620 |  0 NO_ERROR,
  // ---- C620 |  1 MOTOR_CHIP_ACCESS_FAILURE,
  // C610 C620 |  2 MSC_SUPPLY_OVER_VOLTAGE,
  // C610 C620 |  3 THREE_PHASE_CABLE_NOT_CONNECTED,
  // C610 C620 |  4 POSITION_SENSOR_SIGNAL_LOST,
  // ---- C620 |  5 MOTOR_TEMPERATURE_CRITICAL,
  // C610 ---- |  6 MOTOR_STALLED,
  // C610 C620 |  7 MOTOR_CALIBRATION_FAILED,
  // ---- C620 |  8 MOTOR_OVER_TEMPERATURE,
  // C610 C620 | 99 NO_CAN_MESSAGE,
  C6x0ErrorCode getErrorCode() const {
    return manager_.getErrorCode(id_);
  }

  // C610 C620 |  0 NO_ERROR,
  // ---- C620 |  1 MOTOR_CHIP_ACCESS_FAILURE,
  // C610 C620 |  2 MSC_SUPPLY_OVER_VOLTAGE,
  // C610 C620 |  3 THREE_PHASE_CABLE_NOT_CONNECTED,
  // C610 C620 |  4 POSITION_SENSOR_SIGNAL_LOST,
  // ---- C620 |  5 MOTOR_TEMPERATURE_CRITICAL,
  // C610 ---- |  6 MOTOR_STALLED,
  // C610 C620 |  7 MOTOR_CALIBRATION_FAILED,
  // ---- C620 |  8 MOTOR_OVER_TEMPERATURE,
  // C610 C620 | 99 NO_CAN_MESSAGE,
  uint8_t getErrorCodeNum() const {
    return static_cast<uint8_t>(manager_.getErrorCode(id_));
  }

protected:
  C6x0Manager &manager_;
  C6x0Id id_;
};
}  // namespace detail

template<C6x0Type Type>
class C6x0;

template<>
class C6x0<C6x0Type::C610> : public detail::C6x0Base {
public:
  using C6x0Base::C6x0Base;

  float getTorqueCurrent() const {
    return manager_.getCurrentRaw(id_);
  }

  // C610 -10000 to 10000 [mA] (±10A)
  void setCurrent(float current) {
    manager_.setCurrentRefRaw(id_, std::clamp(current, -10000.0f, 10000.0f));
  }
};

template<>
class C6x0<C6x0Type::C620> : public detail::C6x0Base {
public:
  using C6x0Base::C6x0Base;

  float getTorqueCurrent() const {
    return manager_.getCurrentRaw(id_) / 16384.0f * 20000.0f;
  }

  // C620 -20000 to 20000 [mA] (±20A)
  void setCurrent(float current) {
    manager_.setCurrentRefRaw(id_, std::clamp(current, -20000.0f, 20000.0f) / 20000.0f * 16384.0f);
  }
};
