#include "robomas.hpp"
#include "main.h"
#include <algorithm>
#include <cmath>

// 定数
static const float ENCODER_RESOLUTION = 8192.0f;
static const float DEG_PER_COUNT = 360.0f / ENCODER_RESOLUTION;

// initの実装
void RoboMaster::init(uint8_t _id) { this->id = _id; }

// setPIDの実装 (★ここを修正: 変数名をPID.hppに合わせる)
void RoboMaster::setPID(const PIDConfig &cfg) {
  // 速度制御PID
  vel_pid.kp = cfg.speed_kp;
  vel_pid.ki = cfg.speed_ki;
  vel_pid.kd = cfg.speed_kd;

  // ★修正: limit_i -> i_limit
  // (もしエラーが出るならPID.hppを確認して変数名を合わせてください)
  vel_pid.i_limit = cfg.speed_i_limit;

  // ★修正: limit_output -> output_limit
  vel_pid.output_limit = cfg.speed_output_limit;

  // 位置制御PID
  pos_pid.kp = cfg.position_kp;
  pos_pid.ki = cfg.position_ki;
  pos_pid.kd = cfg.position_kd;

  // ★修正
  pos_pid.i_limit = cfg.position_i_limit;
  pos_pid.output_limit = cfg.position_output_limit;

  // 設定変更時はリセット推奨
  vel_pid.reset();
  pos_pid.reset();
}

void RoboMaster::updateFeedback(const uint8_t *can_data) {
  // 1. CANデータから生値を復元
  uint16_t new_angle_raw = (can_data[0] << 8) | can_data[1];
  int16_t new_speed_raw = (can_data[2] << 8) | can_data[3];
  int16_t new_torque_raw = (can_data[4] << 8) | can_data[5];
  uint8_t new_temp = can_data[6];

  // 2. 状態変数の更新
  angle_raw = new_angle_raw;
  current_velocity = (float)new_speed_raw;
  current_torque = new_torque_raw; // 名前修正済み
  temperature = new_temp;

  // 3. 累積角度の計算処理
  if (is_first_update) {
    last_angle_raw = angle_raw;
    total_angle = angle_raw * DEG_PER_COUNT;
    is_first_update = false;
    return;
  }

  int diff = (int)angle_raw - (int)last_angle_raw;
  if (diff > 4096)
    diff -= 8192;
  else if (diff < -4096)
    diff += 8192;

  total_angle += (float)diff * DEG_PER_COUNT;
  last_angle_raw = angle_raw;
  last_feedback_time = HAL_GetTick();
}

int16_t RoboMaster::calculate() {
  if (mode == 3)
    return 0;

  // --- 目標値の補間処理 (Linear Interpolation) ---
  // 毎回 step 分だけ進める
  // オーバーシュートしないようにターゲット到達で止める
  if (interpolation_step > 0) {
    if (current_setpoint < target) {
      current_setpoint += interpolation_step;
      if (current_setpoint > target)
        current_setpoint = target;
    }
  } else if (interpolation_step < 0) {
    if (current_setpoint > target) {
      current_setpoint += interpolation_step; // stepは負の値
      if (current_setpoint < target)
        current_setpoint = target;
    }
  } else {
    // step == 0 の場合は直ちにtargetに一致させる（念のため）
    current_setpoint = target;
  }
  // -------------------------------------------

  float final_current = 0;
  if (mode == 2) {
    float target_velocity = pos_pid.compute(current_setpoint, total_angle);
    final_current = vel_pid.compute(target_velocity, current_velocity);
  } else if (mode == 1) {
    final_current = vel_pid.compute(current_setpoint, current_velocity);
  } else if (mode == 0) {
    // ★修正: limit_output -> output_limit
    final_current = std::max(-vel_pid.output_limit,
                             std::min(current_setpoint, vel_pid.output_limit));
  }
  return (int16_t)final_current;
}

// ★追加: セットポイントリセット
void RoboMaster::resetSetpoint() {
  // モードに応じて初期値を設定
  if (mode == 2) { // 位置制御
    current_setpoint = total_angle;
  } else { // 速度・電流制御
    current_setpoint = 0.0f;
  }
  target = current_setpoint;
  interpolation_step = 0.0f; // 補間なし
}

// ★追加: 新しい目標値を設定し、5回(10ms)で到達するようにstepを計算
void RoboMaster::updateTarget(float new_target) {
  target = new_target;
  // (目標 - 現在) / 5回
  // これにより、次の10ms間に5回の2ms周期で徐々に接近する
  interpolation_step = (target - current_setpoint) / 5.0f;
}
