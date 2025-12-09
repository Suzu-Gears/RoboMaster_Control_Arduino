[English](./README.md) | 日本語

[![Arduino Lint](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml/badge.svg)](https://github.com/Suzu-Gears/RoboMaster_Control_Arduino/actions/workflows/lint.yml)

# RoboMaster_Control_Arduino

## 概要

このリポジトリは、Arduino環境でCANバスを介してDJI RoboMasterのブラシレスDCモーターを扱うためのライブラリ群を収めています。厳密には次の3つのライブラリで構成されています。

- C6x0系ライブラリ（C610/C610 v2/C620/C620 v2用）
- GM6020ライブラリ（GM6020用）
- 混在用ライブラリ

それぞれ用途を分けて利用できるよう設計しています。

## 特徴

- **`C6x0.h`**: C610/C610 v2/C620/C620 v2を制御するためのシンプルなライブラリ。
- **`GM6020.h`**: GM6020モーターを制御するためのシンプルなライブラリ。
- **`RoboMaster_Control.h`**: C6x0 と GM6020 を混在させて管理できるライブラリ。コンパイル時チェック（static_assert）でCAN IDや送信用スロットの競合を検出する。

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

- **C6x0_Multi**: C6x0系の制御例
- **C6x0_Read_Errors**: フィードバック/エラーの読み取り例
- **GM6020_Multi**: GM6020 の電圧/電流モード制御例
- **Mixed_Motors**: C6x0 と GM6020 を混在で扱う例
- **Mixed_Motors_Macro**: Cマクロを使用してモーターのエイリアスを定義する`Mixed_Motors`の別バージョン
- **Compile_Time_Check**: CAN ID 競合検出の例（意図的にコンパイルエラーを発生させる）
