[English](./README.md) | 日本語

[![Arduino Lint](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml/badge.svg)](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml)
[![Ask DeepWiki](https://deepwiki.com/badge.svg)](https://deepwiki.com/Suzu-Gears/RoboMaster_Control_Arduino)

# RoboMaster_Control_Arduino

## 概要

このリポジトリは、Arduino環境でCANバスを介してDJI RoboMasterのブラシレスDCモーターを扱うためのライブラリです。

- C6x0系
  - M2006モーター(ESC C610/C610 v2)
  - M3508モーター(ESC C620/C620 v2)
- GM6020系
  - GM6020電圧制御モード
  - GM6020電流制御モード

に対応しています。

## 特徴

C6x0系のモーターとGM6020を混在して使うことが出来ます。また、フィードバック受信用IDと指令送信用IDの競合を検出できるので、安全にモーターを管理できます。

## 対応ハードウェアと依存関係

このライブラリは`arduino::HardwareCAN`に準拠しています。CAN通信には別途ライブラリが必要です。

| マイコンボード                                  | 必要なCANライブラリ                                     |
| ----------------------------------------------- | ------------------------------------------------------- |
| Arduino (Uno R4 WiFi / Uno R4 Minima / Nano R4) | Arduino_CAN (内蔵)                                      |
| Raspberry Pi Pico (RP2040 / RP2350)             | [RP2040PIO_CAN](https://github.com/eyr1n/RP2040PIO_CAN) |
| ESP32                                           | [ESP32_TWAI](https://github.com/eyr1n/ESP32_TWAI)       |

**注意:** CANトランシーバーが必要です。

## インストール

### 手動インストール

#### Arduino IDEの「.ZIP形式のライブラリをインストール」を使用（推奨）

1.  [GitHubリポジトリ](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/releases/latest)から最新のリリースを`.zip`ファイルとしてダウンロードします。
2.  Arduino IDEで、`スケッチ > ライブラリをインクルード > .ZIP形式のライブラリをインストール...`に移動します。
3.  ダウンロードした`.zip`ファイルを選択します。
4.  Arduino IDEを再起動します。

#### 手動配置

1.  [GitHubリポジトリ](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/releases/latest)から最新のリリースを`.zip`ファイルとしてダウンロードします。
2.  ダウンロードしたファイルを解凍します。
3.  解凍したフォルダをArduinoのライブラリディレクトリ（例：`~/Documents/Arduino/libraries/`）に配置します。
4.  Arduino IDEを再起動します。

## 基本的な使い方

詳細は examples フォルダのサンプルを参照してください。

- **C6x0_Feedback**: C6x0系モーターのフィードバックの読み取り例
- **Mixed_Motors**: C6x0系 と GM6020 を混在させて扱う例

## CAN ID仕様

このライブラリは、DJI RoboMasterのモーターのIDから、プロトコルに準拠したCAN IDを自動的に設定します。IDの競合はライブラリによって検出できますが、設計の参考に以下の表を記載します。

### C6x0系 (C610/C620)

| モーターID | 受信ID (Rx) | 送信ID (Tx) | 送信スロット (Tx Slot) |
| :--------: | :---------: | :---------: | :--------------------: |
| 1          | 0x201       | 0x200       | DATA[0], DATA[1]       |
| 2          | 0x202       | 0x200       | DATA[2], DATA[3]       |
| 3          | 0x203       | 0x200       | DATA[4], DATA[5]       |
| 4          | 0x204       | 0x200       | DATA[6], DATA[7]       |
| 5          | 0x205       | 0x1FF       | DATA[0], DATA[1]       |
| 6          | 0x206       | 0x1FF       | DATA[2], DATA[3]       |
| 7          | 0x207       | 0x1FF       | DATA[4], DATA[5]       |
| 8          | 0x208       | 0x1FF       | DATA[6], DATA[7]       |

### GM6020 (電圧制御モード)

| モーターID | 受信ID (Rx) | 送信ID (Tx) | 送信スロット (Tx Slot) |
| :--------: | :---------: | :---------: | :--------------------: |
| 1          | 0x205       | 0x1FF       | DATA[0], DATA[1]       |
| 2          | 0x206       | 0x1FF       | DATA[2], DATA[3]       |
| 3          | 0x207       | 0x1FF       | DATA[4], DATA[5]       |
| 4          | 0x208       | 0x1FF       | DATA[6], DATA[7]       |
| 5          | 0x209       | 0x2FF       | DATA[0], DATA[1]       |
| 6          | 0x20A       | 0x2FF       | DATA[2], DATA[3]       |
| 7          | 0x20B       | 0x2FF       | DATA[4], DATA[5]       |
| 8(なし)    | なし        | なし        | なし                    |

### GM6020 (電流制御モード)

| モーターID | 受信ID (Rx) | 送信ID (Tx) | 送信スロット (Tx Slot) |
| :--------: | :---------: | :---------: | :--------------------: |
| 1          | 0x205       | 0x1FE       | DATA[0], DATA[1]       |
| 2          | 0x206       | 0x1FE       | DATA[2], DATA[3]       |
| 3          | 0x207       | 0x1FE       | DATA[4], DATA[5]       |
| 4          | 0x208       | 0x1FE       | DATA[6], DATA[7]       |
| 5          | 0x209       | 0x2FE       | DATA[0], DATA[1]       |
| 6          | 0x20A       | 0x2FE       | DATA[2], DATA[3]       |
| 7          | 0x20B       | 0x2FE       | DATA[4], DATA[5]       |
| 8(なし)    | なし        | なし        | なし                    |

## エラーコード

`motor.getErrorCode()` で取得できるエラーコードの一覧。

| コード | Enum名                            | 対象       | 説明                                                         |
| :----: | :-------------------------------- | :--------: | :----------------------------------------------------------- |
| 0      | `NO_ERROR`                        | C610, C620 | 異常なし (正常)                                              |
| 1      | `MOTOR_CHIP_ACCESS_FAILURE`       | C620       | モーターのメモリチップにアクセスできない (電源投入時のセルフテストで検出) |
| 2      | `MSC_SUPPLY_OVER_VOLTAGE`         | C610, C620 | MSC(ESC)の供給電圧が高すぎる (電源投入時のセルフテストで検出)    |
| 3      | `THREE_PHASE_CABLE_NOT_CONNECTED` | C610, C620 | モーターへの三相ケーブルが接続されていない                 |
| 4      | `POSITION_SENSOR_SIGNAL_LOST`     | C610, C620 | モーターに接続された4ピン位置センサーケーブルの信号が喪失 |
| 5      | `MOTOR_TEMPERATURE_CRITICAL`      | C620       | モーター温度が危険域 (例: >= 180°C)                 |
| 6      | `MOTOR_STALLED`                   | C610       | モーターがストールしている                                 |
| 7      | `MOTOR_CALIBRATION_FAILED`        | C610, C620 | モーターのキャリブレーションに失敗                   |
| 8      | `MOTOR_OVER_TEMPERATURE`          | C620       | モーターが過熱している (例: >= 125°C)                      |
| 99     | `NO_CAN_MESSAGE`                  | C610, C620 | (このライブラリの独自定義) CANメッセージ未受信         |
