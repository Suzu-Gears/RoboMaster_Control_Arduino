#pragma once

#include <cstdint>

namespace robomaster {

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

struct SpecBase {
  static constexpr uint32_t rx_id = 0;
  static constexpr uint32_t tx_id = 0;
  static constexpr uint8_t tx_buf_idx = 0;
};

enum class MotorModelType {
  C610,
  C620,
  GM6020_Voltage,
  GM6020_Current
};

struct C610_Model {
  static constexpr float MaxValue = 10000.0f;
  static constexpr float RawScale = 1.0f;
  static constexpr float InverseRawScale = 1.0f / RawScale;
};
struct C620_Model {
  static constexpr float MaxValue = 20000.0f;
  static constexpr float RawScale = 16384.0f / 20000.0f;
  static constexpr float InverseRawScale = 1.0f / RawScale;
};
struct GM6020_Voltage_Model {
  static constexpr float MaxValue = 25000.0f;
  static constexpr float RawScale = 1.0f;
  static constexpr float InverseRawScale = 1.0f / RawScale;
};
struct GM6020_Current_Model {
  static constexpr float MaxValue = 3000.0f;
  static constexpr float RawScale = 16384.0f / 3000.0f;
  static constexpr float InverseRawScale = 1.0f / RawScale;
};

// --- C6x0 Series ---
struct C610 {
  struct ID1 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x201;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 0;
  };
  struct ID2 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x202;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 1;
  };
  struct ID3 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x203;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 2;
  };
  struct ID4 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x204;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 3;
  };
  struct ID5 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x205;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 0;
  };
  struct ID6 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x206;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 1;
  };
  struct ID7 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x207;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 2;
  };
  struct ID8 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C610;
    static constexpr uint32_t rx_id = 0x208;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 3;
  };
};

struct C620 {
  struct ID1 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x201;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 0;
  };
  struct ID2 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x202;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 1;
  };
  struct ID3 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x203;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 2;
  };
  struct ID4 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x204;
    static constexpr uint32_t tx_id = 0x200;
    static constexpr uint8_t tx_buf_idx = 3;
  };
  struct ID5 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x205;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 0;
  };
  struct ID6 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x206;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 1;
  };
  struct ID7 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x207;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 2;
  };
  struct ID8 : SpecBase {
    static constexpr MotorModelType model_type = MotorModelType::C620;
    static constexpr uint32_t rx_id = 0x208;
    static constexpr uint32_t tx_id = 0x1FF;
    static constexpr uint8_t tx_buf_idx = 3;
  };
};

// --- GM6020 Series ---
struct GM6020 {
  struct Voltage {
    struct ID1 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x205;
      static constexpr uint32_t tx_id = 0x1FF;
      static constexpr uint8_t tx_buf_idx = 0;
    };
    struct ID2 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x206;
      static constexpr uint32_t tx_id = 0x1FF;
      static constexpr uint8_t tx_buf_idx = 1;
    };
    struct ID3 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x207;
      static constexpr uint32_t tx_id = 0x1FF;
      static constexpr uint8_t tx_buf_idx = 2;
    };
    struct ID4 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x208;
      static constexpr uint32_t tx_id = 0x1FF;
      static constexpr uint8_t tx_buf_idx = 3;
    };
    struct ID5 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x209;
      static constexpr uint32_t tx_id = 0x2FF;
      static constexpr uint8_t tx_buf_idx = 0;
    };
    struct ID6 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x20A;
      static constexpr uint32_t tx_id = 0x2FF;
      static constexpr uint8_t tx_buf_idx = 1;
    };
    struct ID7 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Voltage;
      static constexpr uint32_t rx_id = 0x20B;
      static constexpr uint32_t tx_id = 0x2FF;
      static constexpr uint8_t tx_buf_idx = 2;
    };
  };

  struct Current {
    struct ID1 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x205;
      static constexpr uint32_t tx_id = 0x1FE;
      static constexpr uint8_t tx_buf_idx = 0;
    };
    struct ID2 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x206;
      static constexpr uint32_t tx_id = 0x1FE;
      static constexpr uint8_t tx_buf_idx = 1;
    };
    struct ID3 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x207;
      static constexpr uint32_t tx_id = 0x1FE;
      static constexpr uint8_t tx_buf_idx = 2;
    };
    struct ID4 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x208;
      static constexpr uint32_t tx_id = 0x1FE;
      static constexpr uint8_t tx_buf_idx = 3;
    };
    struct ID5 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x209;
      static constexpr uint32_t tx_id = 0x2FE;
      static constexpr uint8_t tx_buf_idx = 0;
    };
    struct ID6 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x20A;
      static constexpr uint32_t tx_id = 0x2FE;
      static constexpr uint8_t tx_buf_idx = 1;
    };
    struct ID7 : SpecBase {
      static constexpr MotorModelType model_type = MotorModelType::GM6020_Current;
      static constexpr uint32_t rx_id = 0x20B;
      static constexpr uint32_t tx_id = 0x2FE;
      static constexpr uint8_t tx_buf_idx = 2;
    };
  };
};

}  // namespace robomaster
