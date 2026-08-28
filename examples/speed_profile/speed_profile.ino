#include <RP2040PIO_CAN.h>

const uint32_t CAN_TX_PIN = 0;
const uint32_t CAN_RX_PIN = 1;
const uint32_t BUTTON_PIN = 14;  // ボタンを接続するGPIOピン

const float KP = 50.0f;
const float KI = 685.0f;

#include <RoboMaster_Control.h>
#include "pid_controller.hpp"
#include "gen_data.hpp"

// ログデータ用構造体
struct Log {
  float targetVelocity;
  float currentVelocity;
  float currentRad;
};
const int LOG_SIZE = 1000;
Log logs[LOG_SIZE];
volatile bool log_ready_to_print = false;

PidController motor_pid;

using namespace robomaster;

RoboMasterManager manager(&CAN);
C610 motor_c610(manager, C6x0Id::ID_1);

static repeating_timer_t timer;

volatile uint32_t tick = 0;
volatile bool profile_running = false;
unsigned long last_button_press = 0;

// ボタン割り込み。チャタリング防止付き。
void button_isr() {
  unsigned long now = millis();
  if (now - last_button_press > 200) {  // 200msのデバウンス
    last_button_press = now;
    if (!profile_running) {
      // ログ配列をクリア
      for (int i = 0; i < LOG_SIZE; i++) {
        logs[i] = { 0.0f, 0.0f, 0.0f };
      }
      tick = 0;
      profile_running = true;
    }
  }
}

// 1kHzで呼び出される割り込み関数 1kHz以内に処理を終えたい printとか入れないほうがいい
bool timer_int(repeating_timer_t *t) {
  manager.update();

  if (profile_running) {
    if (tick >= xbar_ref.size()) {
      profile_running = false;    // プロファイルの終端に達したら停止
      log_ready_to_print = true;  // ログ出力の準備ができたことを通知
      motor_c610.setCurrent(0.0f);
    } else {
      const float target_velocity = xbar_ref[tick];
      const float current_velocity = motor_c610.getVelocityRad();  // 現在の速度を取得

      // ログを記録 (最初の1000サンプルのみ)
      if (tick < LOG_SIZE) {
        logs[tick].targetVelocity = target_velocity;
        logs[tick].currentVelocity = current_velocity;
        logs[tick].currentRad = motor_c610.getAccumPositionRad() / 36.0f;
      }
      const float output_current = motor_pid.solve(target_velocity - current_velocity);

      motor_c610.setCurrent(output_current);
      tick++;
    }
  } else {
    motor_c610.setCurrent(0.0f);  // 実行中でなければモーターを停止
  }

  manager.transmit();

  return (true);
}

void setup() {
  Serial.begin(115200);

  CAN.setTX(CAN_TX_PIN);
  CAN.setRX(CAN_RX_PIN);
  CAN.begin(CanBitRate::BR_1000k);

  motor_pid.init({ 0.001f, KP, KI, 0.0f, 0.0f });

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), button_isr, FALLING);

  add_repeating_timer_us(-1000, timer_int, NULL, &timer);  // interval setting 1kHzでtimer_int関数を呼び出す
}

void loop() {
  if (log_ready_to_print) {
    Serial.println("Target,Current,Rad");
    for (int i = 0; i < LOG_SIZE; i++) {
      Serial.print(logs[i].targetVelocity, 6);
      Serial.print(",");
      Serial.print(logs[i].currentVelocity, 6);
      Serial.print(",");
      Serial.println(logs[i].currentRad, 6);
    }
    log_ready_to_print = false;  // ログは一度だけ出力
  }
}
