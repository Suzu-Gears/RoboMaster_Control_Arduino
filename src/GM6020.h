#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>

#if !__has_include(<api/HardwareCAN.h>)
#error "GM6020.h: include a HardwareCAN provider (e.g. <Arduino_CAN.h>, <ESP32_TWAI.h>, <RP2040PIO_CAN.h>) before this file."
#endif

#include <api/HardwareCAN.h>

enum class GM6020Type {
  Voltage,
  Current,
};

enum class GM6020Id {
  ID_1,
  ID_2,
  ID_3,
  ID_4,
  ID_5,
  ID_6,
  ID_7
};

class GM6020Manager {
public:
  GM6020Manager(arduino::HardwareCAN *can) : can_{ can } {}

  void update() {
    while (can_->available()) {
      CanMsg msg = can_->read();
      uint32_t id = msg.getStandardId();
      if (0x205 <= id && id <= 0x20B) {
        Params &motor = params_[id - 0x205];
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
        // data[7] is none
      }
    }
  }

  bool transmit() {
    if (!transmit_voltage(0x1FF, 0, 4)) return false;
    if (!transmit_voltage(0x2FF, 4, 3)) return false;
    if (!transmit_current(0x1FE, 0, 4)) return false;
    if (!transmit_current(0x2FE, 4, 3)) return false;
    return true;
  }

  int16_t getPosition(GM6020Id id) const {
    return params_[toUnderlying(id)].prev_position_.value_or(0);
  }

  int64_t getAccumPosition(GM6020Id id) const {
    return params_[toUnderlying(id)].position_;
  }

  int16_t getRpm(GM6020Id id) const {
    return params_[toUnderlying(id)].rpm_;
  }

  int16_t getCurrentRaw(GM6020Id id) const {
    return params_[toUnderlying(id)].current_raw_;
  }

  int8_t getTemperature(GM6020Id id) const {
    return params_[toUnderlying(id)].temperature;
  }

  void setVoltageRefRaw(GM6020Id id, int16_t voltage) {
    params_[toUnderlying(id)].voltage_ref_ = voltage;
    params_[toUnderlying(id)].current_ref_.reset();  // Exclusive
  }

  void setCurrentRefRaw(GM6020Id id, int16_t current) {
    params_[toUnderlying(id)].current_ref_ = current;
    params_[toUnderlying(id)].voltage_ref_.reset();  // Exclusive
  }

private:
  bool transmit_voltage(uint32_t can_id, size_t start_index, size_t count) {
    return transmit_frame(can_id, start_index, count, [](const Params &p) {
      return p.voltage_ref_;
    });
  }

  bool transmit_current(uint32_t can_id, size_t start_index, size_t count) {
    return transmit_frame(can_id, start_index, count, [](const Params &p) {
      return p.current_ref_;
    });
  }

  template<typename F>
  bool transmit_frame(uint32_t can_id, size_t start_index, size_t count, F getValue) {
    if (std::any_of(params_.begin() + start_index, params_.begin() + start_index + count, [&](const Params &p) {
          return getValue(p).has_value();
        })) {
      CanMsg msg;
      msg.id = CanStandardId(can_id);
      msg.data_length = 8;
      for (size_t i = 0; i < 4; ++i) {
        const int16_t val_ref = (i < count) ? getValue(params_[start_index + i]).value_or(0) : 0;
        msg.data[i * 2] = val_ref >> 8;
        msg.data[i * 2 + 1] = val_ref;
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
    std::optional<int16_t> voltage_ref_;
    std::optional<int16_t> current_ref_;
    int8_t temperature = 0;
  };

  arduino::HardwareCAN *can_;
  std::array<Params, 7> params_{};

  template<class T>
  constexpr typename std::underlying_type<T>::type
  toUnderlying(T value) const noexcept {
    return static_cast<typename std::underlying_type<T>::type>(value);
  }
};

namespace detail {
class GM6020Base {
public:
  GM6020Base(GM6020Manager &manager, GM6020Id id)
    : manager_{ manager }, id_{ id } {}

  GM6020Base(const GM6020Base &) = delete;
  GM6020Base &operator=(const GM6020Base &) = delete;

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

  // [mA]
  int16_t getTorqueCurrent() const {
    return manager_.getCurrentRaw(id_);
  }

  // [°C]
  int8_t getTemperature() const {
    return manager_.getTemperature(id_);
  }

protected:
  GM6020Manager &manager_;
  GM6020Id id_;
};
}  // namespace detail

template<GM6020Type Mode>
class GM6020;

template<>
class GM6020<GM6020Type::Voltage> : public detail::GM6020Base {
public:
  GM6020(GM6020Manager &manager, GM6020Id id) : GM6020Base(manager, id) {}

  ~GM6020() {
    manager_.setVoltageRefRaw(id_, 0);
  }

  // -25000 to 25000 [mV] (±25V)
  void setVoltage(float voltage) {
    manager_.setVoltageRefRaw(id_, std::clamp(voltage, -25000.0f, 25000.0f));
  }
};

template<>
class GM6020<GM6020Type::Current> : public detail::GM6020Base {
public:
  GM6020(GM6020Manager &manager, GM6020Id id) : GM6020Base(manager, id) {}

  ~GM6020() {
    manager_.setCurrentRefRaw(id_, 0);
  }

  // -3000 to 3000 [mA] (±3A)
  void setCurrent(float current) {
    manager_.setCurrentRefRaw(id_, std::clamp(current, -3000.0f, 3000.0f) / 3000.0f * 16384.0f);
  }
};
