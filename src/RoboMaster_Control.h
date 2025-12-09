#pragma once

#include <algorithm>
#include <array>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>

#if !__has_include(<api/HardwareCAN.h>)
#error "RoboMaster_Control.h requires a HardwareCAN compatible library to be included first (e.g. <Arduino_CAN.h>)."
#endif

#include <api/HardwareCAN.h>

#include "RoboMaster_Types.h"
#include "RoboMaster_Check.h"

namespace robomaster {

template<typename... AliasSpecs> class Manager;  // Forward declaration

template<typename Spec>
class Motor {
private:
  template<typename... AliasSpecs> friend class Manager;

  struct Params {
    int64_t position_ = 0;
    std::optional<int16_t> prev_position_;
    int16_t rpm_ = 0;
    int16_t current_raw_ = 0;
    std::optional<int16_t> value_ref_;
    int8_t temperature = 0;
    C6x0ErrorCode error_code = C6x0ErrorCode::NO_CAN_MESSAGE;
  };

  Params params_;

  void set_target_internal(float value) {
    float max_val = 0;
    float scale = 1.0f;
    if constexpr (Spec::model_type == MotorModelType::C610) {
      max_val = C610_Model::MaxValue;
      scale = C610_Model::RawScale;
    } else if constexpr (Spec::model_type == MotorModelType::C620) {
      max_val = C620_Model::MaxValue;
      scale = C620_Model::RawScale;
    } else if constexpr (Spec::model_type == MotorModelType::GM6020_Voltage) {
      max_val = GM6020_Voltage_Model::MaxValue;
      scale = GM6020_Voltage_Model::RawScale;
    } else if constexpr (Spec::model_type == MotorModelType::GM6020_Current) {
      max_val = GM6020_Current_Model::MaxValue;
      scale = GM6020_Current_Model::RawScale;
    }
    value = std::clamp(value, -max_val, max_val);
    params_.value_ref_ = static_cast<int16_t>(value * scale);
  }

public:
  Motor() = default;

  // Angle range: 0 to 8191 (0° to 360°)
  int16_t getPosition() const {
    return params_.prev_position_.value_or(0);
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
    return params_.position_;
  }

  float getAccumPositionRad() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 2.0f * M_PI;
  }

  float getAccumPositionDeg() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 360.0f;
  }

  int16_t getRpm() const {
    return params_.rpm_;
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
    if constexpr (Spec::model_type == MotorModelType::C610) {
      return static_cast<int16_t>(params_.current_raw_ / C610_Model::RawScale);
    } else if constexpr (Spec::model_type == MotorModelType::C620) {
      return static_cast<int16_t>(params_.current_raw_ / C620_Model::RawScale);
    } else if constexpr (Spec::model_type == MotorModelType::GM6020_Voltage) {
      return static_cast<int16_t>(params_.current_raw_ / GM6020_Voltage_Model::RawScale);
    } else if constexpr (Spec::model_type == MotorModelType::GM6020_Current) {
      return static_cast<int16_t>(params_.current_raw_ / GM6020_Current_Model::RawScale);
    }
    return 0;
  }

  // [°C]
  int8_t getTemperature() const {
    return params_.temperature;
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
  template<typename T = Spec, std::enable_if_t< T::model_type == MotorModelType::C610 || T::model_type == MotorModelType::C620, std::nullptr_t> = nullptr>
  C6x0ErrorCode getErrorCode() const {
    return params_.error_code;
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
  template<typename T = Spec, std::enable_if_t< T::model_type == MotorModelType::C610 || T::model_type == MotorModelType::C620, std::nullptr_t> = nullptr>
  uint8_t getErrorCodeNum() const {
    return static_cast<uint8_t>(params_.error_code);
  }

  // C610 -10000 to 10000 [mA] (±10A)
  template<typename T = Spec, std::enable_if_t<T::model_type == MotorModelType::C610, std::nullptr_t> = nullptr>
  void setCurrent(float current_mA) {
    set_target_internal(current_mA);
  }

  // C620 -20000 to 20000 [mA] (±20A)
  template<typename T = Spec, std::enable_if_t<T::model_type == MotorModelType::C620, std::nullptr_t> = nullptr>
  void setCurrent(float current_mA) {
    set_target_internal(current_mA);
  }

  // GM6020 -3000 to 3000 [mA] (±3A)
  template<typename T = Spec, std::enable_if_t<T::model_type == MotorModelType::GM6020_Current, std::nullptr_t> = nullptr>
  void setCurrent(float current_mA) {
    set_target_internal(current_mA);
  }

  // GM6020 -25000 to 25000 [mV] (±25V)
  template<typename T = Spec, std::enable_if_t< T::model_type == MotorModelType::GM6020_Voltage, std::nullptr_t> = nullptr>
  void setVoltage(float voltage_mV) {
    set_target_internal(voltage_mV);
  }
};

template<typename... AliasSpecs>
class Manager {
  static_assert(!Check::has_rx_conflict<AliasSpecs...>(), "[RoboMaster] Rx ID Conflict: Multiple motors share the same feedback CAN ID.");
  static_assert(!Check::has_tx_conflict<AliasSpecs...>(), "[RoboMaster] Tx Message Slot Conflict: Multiple motors share the same CAN command slot.");

public:
  Manager(arduino::HardwareCAN* can) : can_(can), params_{} {}

  template<typename AliasSpec>
  Motor<typename AliasSpec::spec>& get() {
    return std::get<motor_index<AliasSpec>()>(params_);
  }

  void update() {
    while (can_->available()) {
      CanMsg msg = can_->read();
      for_each_motor([&](auto& motor, auto alias_proxy) {
        using Spec = typename decltype(alias_proxy)::type::spec;
        if (Spec::rx_id == msg.getStandardId()) {
          const int16_t position = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
          if (motor.params_.prev_position_) {
            const int16_t delta = position - *motor.params_.prev_position_;
            motor.params_.position_ += ((delta + 4096) & 8191) - 4096;
          } else {
            motor.params_.position_ = position;
          }
          motor.params_.prev_position_ = position;
          motor.params_.rpm_ = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
          motor.params_.current_raw_ = static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
          motor.params_.temperature = static_cast<int8_t>(msg.data[6]);
          if constexpr (Spec::model_type == MotorModelType::C610 || Spec::model_type == MotorModelType::C620) {
            motor.params_.error_code = static_cast<C6x0ErrorCode>(msg.data[7]);
          }
        }
      });
    }
  }

  bool transmit() {
    if (!transmit_group(0x200)) return false;
    if (!transmit_group(0x1FF)) return false;
    if (!transmit_group(0x2FF)) return false;
    if (!transmit_group(0x1FE)) return false;
    if (!transmit_group(0x2FE)) return false;
    return true;
  }

private:
  arduino::HardwareCAN* can_;
  std::tuple<Motor<typename AliasSpecs::spec>...> params_;

  bool transmit_group(uint32_t tx_id) {
    bool should_send = false;
    std::array<uint8_t, 8> data{};

    for_each_motor([&](auto& motor, auto alias_proxy) {
      using Spec = typename decltype(alias_proxy)::type::spec;
      if (Spec::tx_id == tx_id) {
        if (const auto& target = motor.params_.value_ref_; target.has_value()) {
          should_send = true;
          int16_t val = target.value();
          data[Spec::tx_buf_idx * 2] = val >> 8;
          data[Spec::tx_buf_idx * 2 + 1] = val;
        }
      }
    });

    if (should_send) {
      CanMsg msg;
      msg.id = CanStandardId(tx_id);
      msg.data_length = 8;
      for (int i = 0; i < 8; ++i) msg.data[i] = data[i];
      return can_->write(msg) >= 0;
    }
    return true;
  }

  template<typename T> struct TypeProxy {
    using type = T;
  };

  template<typename Func, std::size_t... Is>
  void for_each_motor_impl(Func func, std::index_sequence<Is...>) {
    (func(std::get<Is>(params_), TypeProxy<typename std::tuple_element<Is, std::tuple<AliasSpecs...>>::type>{}), ...);
  }

  template<typename Func> void for_each_motor(Func func) {
    for_each_motor_impl(func, std::make_index_sequence<sizeof...(AliasSpecs)>{});
  }

  template<typename Alias, size_t I = 0>
  static constexpr size_t motor_index() {
    if constexpr (std::is_same_v<Alias, typename std::tuple_element<I, std::tuple<AliasSpecs...>>::type>) return I;
    else return motor_index<Alias, I + 1>();
  }
};

}  // namespace robomaster
