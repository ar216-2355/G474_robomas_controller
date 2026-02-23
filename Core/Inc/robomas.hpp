#pragma once
#include "PID.hpp"
#include "usb_packet.hpp" // PIDConfig構造体を使うために必要
#include <cstdint>

class RoboMaster {
public:
  uint8_t id = 0;
  uint8_t mode = 3;
  float target = 0;

  PIDController vel_pid;
  PIDController pos_pid;

  // センサー情報
  float current_velocity = 0.0f; // rpm

  // ★修正: app.cppに合わせて変数名を変更 (torque_current_raw -> current_torque)
  int16_t current_torque = 0; // トルク電流 (生値: -16384 ~ 16384)

  uint8_t temperature = 0; // ℃

  // 角度関連 (多回転対応)
  uint16_t angle_raw = 0;      // 現在の生値 (0-8191)
  uint16_t last_angle_raw = 0; // 前回の生値
  float total_angle = 0.0f;    // 累積角度

  bool is_first_update = true; // 初回受信フラグ
  uint32_t last_feedback_time = 0;

  // メソッド宣言
  void updateFeedback(const uint8_t *can_data);
  int16_t calculate();

  // ★追加: 初期化メソッド
  void init(uint8_t id);

  // ★追加: PID設定メソッド
  void setPID(const PIDConfig &config);

  // ★追加: 目標値ランプ(補間)用変数
  float current_setpoint = 0.0f;
  float interpolation_step = 0.0f; // 1回(2ms)ごとの変化量

  // ★追加: セットポイントリセット
  void resetSetpoint();

  // ★追加: 新しい目標値をセットし、補間ステップを計算する
  void updateTarget(float new_target);
};
