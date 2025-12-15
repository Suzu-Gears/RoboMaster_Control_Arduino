// This code is based on https://github.com/tutrc-freshman/TUTRC_ArduinoLib/blob/master/src/C6x0.h and https://github.com/eyr1n/halx_driver/blob/master/include/halx/driver/c6x0.hpp

#pragma once

#include <Arduino.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>

#if !__has_include(<api/HardwareCAN.h>)
#error "RoboMaster_Control.h requires a HardwareCAN compatible library to be included first (e.g. <Arduino_CAN.h>)."
#endif

#include <api/HardwareCAN.h>

namespace robomaster {

enum class MotorType {
  C610,
  C620,
  GM6020_Voltage,
  GM6020_Current
};

enum class C6x0Id {
  ID_1 = 1,
  ID_2,
  ID_3,
  ID_4,
  ID_5,
  ID_6,
  ID_7,
  ID_8
};

enum class GM6020Id {
  ID_1 = 1,
  ID_2,
  ID_3,
  ID_4,
  ID_5,
  ID_6,
  ID_7
};

// Priority: Smaller value first.
enum C6x0ErrorCode : uint8_t {
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

inline const char* getMotorTypeString(MotorType type) {
  switch (type) {
    case MotorType::C610: return "C610";
    case MotorType::C620: return "C620";
    case MotorType::GM6020_Voltage: return "GM6020_Voltage";
    case MotorType::GM6020_Current: return "GM6020_Current";
    default: return "UNKNOWN_MOTOR_TYPE";
  }
}

inline const char* getC6x0ErrorCodeString(C6x0ErrorCode code) {
  switch (code) {
    case C6x0ErrorCode::NO_ERROR: return "NO_ERROR";
    case C6x0ErrorCode::MOTOR_CHIP_ACCESS_FAILURE: return "MOTOR_CHIP_ACCESS_FAILURE";
    case C6x0ErrorCode::MSC_SUPPLY_OVER_VOLTAGE: return "MSC_SUPPLY_OVER_VOLTAGE";
    case C6x0ErrorCode::THREE_PHASE_CABLE_NOT_CONNECTED: return "THREE_PHASE_CABLE_NOT_CONNECTED";
    case C6x0ErrorCode::POSITION_SENSOR_SIGNAL_LOST: return "POSITION_SENSOR_SIGNAL_LOST";
    case C6x0ErrorCode::MOTOR_TEMPERATURE_CRITICAL: return "MOTOR_TEMPERATURE_CRITICAL";
    case C6x0ErrorCode::MOTOR_STALLED: return "MOTOR_STALLED";
    case C6x0ErrorCode::MOTOR_CALIBRATION_FAILED: return "MOTOR_CALIBRATION_FAILED";
    case C6x0ErrorCode::MOTOR_OVER_TEMPERATURE: return "MOTOR_OVER_TEMPERATURE";
    case C6x0ErrorCode::NO_CAN_MESSAGE: return "NO_CAN_MESSAGE";
    default: return "UNKNOWN_ERROR_CODE";
  }
}

namespace detail {
class RoboMasterManagerBase {
public:
  struct MotorConfig {
    uint8_t id;
    MotorType motor_type;
    uint16_t rx_id;
    uint16_t tx_id;
    uint8_t tx_buf_idx;
  };

  struct MotorParams {
    std::optional<int16_t> prev_position_;
    int64_t accum_position_ = 0;
    int16_t rpm_ = 0;
    int16_t current_raw_ = 0;
    std::optional<int16_t> target_raw_;
    int8_t temp_ = 0;
    C6x0ErrorCode error_code_ = C6x0ErrorCode::NO_CAN_MESSAGE;
  };

  virtual ~RoboMasterManagerBase() = default;

  virtual void update() = 0;
  virtual bool transmit() = 0;
  virtual size_t getMotorCount() const = 0;
  virtual bool hasConflict() const = 0;
  virtual std::optional<MotorConfig> getConflictInfo() const = 0;
  virtual const MotorConfig& getMotorConfig(size_t index) const = 0;

  virtual size_t registerMotor(MotorType motor, uint8_t id) = 0;
  virtual void setTargetRaw(size_t motor_idx, int16_t target_raw) = 0;
  virtual const MotorParams& getParams(size_t motor_idx) const = 0;
};

class RoboMasterMotor {
public:
  // Angle range: 0 to 8191 (0° to 360°)
  int16_t getPositionRaw() const {
    return manager_.getParams(motor_idx_).prev_position_.value_or(0);
  }
  // Unit is encoder ticks (8192 ticks per revolution)
  int64_t getAccumPosition() const {
    return manager_.getParams(motor_idx_).accum_position_;
  }
  int16_t getRpm() const {
    return manager_.getParams(motor_idx_).rpm_;
  }
  // [°C]
  int8_t getTemperature() const {
    return manager_.getParams(motor_idx_).temp_;
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
    return manager_.getParams(motor_idx_).error_code_;
  }
  // Caution: Directly sets raw target; no clamping. Invalid values can cause motor damage.
  void setTargetRaw(int16_t target_raw) {
    manager_.setTargetRaw(motor_idx_, target_raw);
  }
  // [mA]
  virtual float getTorqueCurrent() const {
    return manager_.getParams(motor_idx_).current_raw_;
  }

  // Angle range: 0 to 2PI [rad]
  float getPositionRad() const {
    return static_cast<float>(getPositionRaw()) / 8192.0f * 2.0f * PI;
  }
  // Angle range: 0 to 360 [deg]
  float getPositionDeg() const {
    return static_cast<float>(getPositionRaw()) / 8192.0f * 360.0f;
  }
  float getAccumPositionRad() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 2.0f * PI;
  }
  float getAccumPositionDeg() const {
    return static_cast<float>(getAccumPosition()) / 8192.0f * 360.0f;
  }
  float getRps() const {
    return static_cast<float>(getRpm()) / 60.0f;
  }
  // [rad/s]
  float getVelocityRad() const {
    return static_cast<float>(getRpm()) / 60.0f * 2.0f * PI;
  }

protected:
  RoboMasterMotor(RoboMasterManagerBase& manager, size_t motor_idx) : manager_(manager), motor_idx_(motor_idx) {}
  RoboMasterManagerBase& manager_;
  const size_t motor_idx_;
};

}  // namespace detail


template<size_t MaxMotors = 15>
class RoboMasterManager : public detail::RoboMasterManagerBase {
public:
  RoboMasterManager(arduino::HardwareCAN* can) : can_(can), motor_count_(0), conflict_info_(std::nullopt) {}

  void update() override {
    while (can_->available()) {
      CanMsg msg = can_->read();
      const uint32_t msg_id = msg.getStandardId();
      for (size_t i = 0; i < motor_count_; ++i) {
        if (configs_[i].rx_id == msg_id) {
          auto& param = params_[i];
          const int16_t position_raw = static_cast<int16_t>(msg.data[0] << 8 | msg.data[1]);
          if (param.prev_position_) {
            const int16_t delta = position_raw - *param.prev_position_;
            param.accum_position_ += ((delta + 4096) & 8191) - 4096;
          } else {
            param.accum_position_ = position_raw;
          }
          param.prev_position_ = position_raw;
          param.rpm_ = static_cast<int16_t>(msg.data[2] << 8 | msg.data[3]);
          param.current_raw_ = static_cast<int16_t>(msg.data[4] << 8 | msg.data[5]);
          param.temp_ = static_cast<int8_t>(msg.data[6]);
          if (configs_[i].motor_type == MotorType::C610 || configs_[i].motor_type == MotorType::C620) {
            param.error_code_ = static_cast<C6x0ErrorCode>(msg.data[7]);
          }
          break;
        }
      }
    }
  }

  bool transmit() override {
    if (!sendMotorControlCommands(0x200)) return false;
    if (!sendMotorControlCommands(0x1FF)) return false;
    if (!sendMotorControlCommands(0x2FF)) return false;
    if (!sendMotorControlCommands(0x1FE)) return false;
    if (!sendMotorControlCommands(0x2FE)) return false;
    return true;
  }

  size_t getMotorCount() const override {
    return motor_count_;
  }

  bool hasConflict() const override {
    return conflict_info_.has_value();
  }

  std::optional<MotorConfig> getConflictInfo() const override {
    return conflict_info_;
  }

  const MotorConfig& getMotorConfig(size_t index) const override {
    return configs_[index];
  }

  size_t registerMotor(MotorType motor, uint8_t id) override {
    if (hasConflict() || motor_count_ >= MaxMotors) return -1;
    uint16_t rx_id = 0, tx_id = 0;
    uint8_t tx_buf_idx = 0;
    resolveCanId(motor, id, rx_id, tx_id, tx_buf_idx);
    MotorConfig motor_config;
    motor_config.id = id;
    motor_config.motor_type = motor;
    motor_config.rx_id = rx_id;
    motor_config.tx_id = tx_id;
    motor_config.tx_buf_idx = tx_buf_idx;
    for (size_t i = 0; i < motor_count_; ++i) {
      if (configs_[i].rx_id == rx_id || (configs_[i].tx_id == tx_id && configs_[i].tx_buf_idx == tx_buf_idx)) {
        conflict_info_ = motor_config;
        return -1;
      }
    }
    const size_t motor_idx = motor_count_++;
    configs_[motor_idx] = motor_config;
    params_[motor_idx] = {};
    return motor_idx;
  }

  void setTargetRaw(size_t motor_idx, int16_t target_raw) override {
    if (motor_idx < motor_count_) {
      params_[motor_idx].target_raw_ = target_raw;
    }
  }

  const MotorParams& getParams(size_t motor_idx) const override {
    return params_[motor_idx];
  }

private:
  static void resolveCanId(MotorType motor_type, uint8_t id_val, uint16_t& rx_id, uint16_t& tx_id, uint8_t& tx_buf_idx) {
    tx_buf_idx = (id_val <= 4) ? (id_val - 1) : (id_val - 5);
    if (motor_type == MotorType::C610 || motor_type == MotorType::C620) {
      rx_id = 0x200 + id_val;
      tx_id = (id_val <= 4) ? 0x200 : 0x1FF;
    } else {
      rx_id = 0x204 + id_val;
      tx_id = (id_val <= 4) ? 0x1FE : 0x2FE;
      if (motor_type == MotorType::GM6020_Voltage) {
        tx_id++;
      }
    }
  }

  bool sendMotorControlCommands(uint32_t tx_id) {
    CanMsg msg;
    msg.id = CanStandardId(tx_id);
    msg.data_length = 8;
    bool should_send = false;
    for (size_t i = 0; i < motor_count_; ++i) {
      if (configs_[i].tx_id == tx_id && params_[i].target_raw_.has_value()) {
        should_send = true;
        int16_t raw_val = params_[i].target_raw_.value();
        msg.data[configs_[i].tx_buf_idx * 2] = raw_val >> 8;
        msg.data[configs_[i].tx_buf_idx * 2 + 1] = raw_val;
      }
    }
    if (!should_send) {
      return true;
    }
    return can_->write(msg) >= 0;
  }

  arduino::HardwareCAN* can_;
  size_t motor_count_;
  std::optional<MotorConfig> conflict_info_;
  std::array<MotorConfig, MaxMotors> configs_{};
  std::array<MotorParams, MaxMotors> params_{};
};

class C610 : public detail::RoboMasterMotor {
public:
  C610(detail::RoboMasterManagerBase& manager, C6x0Id id) : detail::RoboMasterMotor(manager, manager.registerMotor(MotorType::C610, static_cast<uint8_t>(id))) {}
  // C610: -10000 to 10000 [mA] (±10A) (Clamped)
  void setCurrent(float current_mA) {
    manager_.setTargetRaw(motor_idx_, std::clamp(current_mA, -10000.0f, 10000.0f));
  }
};

class C620 : public detail::RoboMasterMotor {
public:
  C620(detail::RoboMasterManagerBase& manager, C6x0Id id) : detail::RoboMasterMotor(manager, manager.registerMotor(MotorType::C620, static_cast<uint8_t>(id))) {}
  // [mA]
  float getTorqueCurrent() const override {
    return static_cast<float>(manager_.getParams(motor_idx_).current_raw_) / 16384.0f * 20000.0f;
  }
  // C620: -20000 to 20000 [mA] (±20A) (Clamped)
  void setCurrent(float current_mA) {
    int16_t target = std::clamp(current_mA / 20000.0f * 16384.0f, -16384.0f, 16384.0f);
    manager_.setTargetRaw(motor_idx_, target);
  }
};

class GM6020_Voltage : public detail::RoboMasterMotor {
public:
  GM6020_Voltage(detail::RoboMasterManagerBase& manager, GM6020Id id) : detail::RoboMasterMotor(manager, manager.registerMotor(MotorType::GM6020_Voltage, static_cast<uint8_t>(id))) {}
  // GM6020: -25000 to 25000 [mV] (±25V) (Clamped)
  void setVoltage(float voltage_mV) {
    manager_.setTargetRaw(motor_idx_, std::clamp(voltage_mV, -25000.0f, 25000.0f));
  }
};

class GM6020_Current : public detail::RoboMasterMotor {
public:
  GM6020_Current(detail::RoboMasterManagerBase& manager, GM6020Id id) : detail::RoboMasterMotor(manager, manager.registerMotor(MotorType::GM6020_Current, static_cast<uint8_t>(id))) {}
  // GM6020: -3000 to 3000 [mA] (±3A) (Clamped)
  void setCurrent(float current_mA) {
    int16_t target = std::clamp(current_mA / 3000.0f * 16384.0f, -16384.0f, 16384.0f);
    manager_.setTargetRaw(motor_idx_, target);
  }
};

}  // namespace robomaster
