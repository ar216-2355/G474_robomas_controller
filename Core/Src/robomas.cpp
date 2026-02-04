#include "robomas.hpp"
#include <cmath> // std::abs用
#include <algorithm> // std::max, std::min 用
#include "main.h"

// 定数: エンコーダ分解能 (0~8191)
static const float ENCODER_RESOLUTION = 8192.0f;
static const float DEG_PER_COUNT = 360.0f / ENCODER_RESOLUTION;

void RoboMaster::updateFeedback(const uint8_t* can_data) {
    // 1. CANデータから生値を復元 (ビッグエンディアン)
    // Data[0]: Angle High, Data[1]: Angle Low
    uint16_t new_angle_raw = (can_data[0] << 8) | can_data[1];
    // Data[2]: Speed High, Data[3]: Speed Low
    int16_t new_speed_raw  = (can_data[2] << 8) | can_data[3];
    // Data[4]: Torque High, Data[5]: Torque Low
    int16_t new_torque_raw = (can_data[4] << 8) | can_data[5];
    // Data[6]: Temp
    uint8_t new_temp = can_data[6];


    // 2. 状態変数の更新
    angle_raw = new_angle_raw;
    current_velocity = (float)new_speed_raw; // 必要ならギア比で割る
    torque_current_raw = new_torque_raw;
    temperature = new_temp;


    // 3. ★累積角度の計算処理 (ここが核心) ★
    if (is_first_update) {
        // 初回は「前回値」がないので、現在の値を基準にする
        last_angle_raw = angle_raw;
        total_angle = angle_raw * DEG_PER_COUNT; // 初期角度をセット(または0にする)
        is_first_update = false;
        return;
    }

    // 差分を計算
    int diff = (int)angle_raw - (int)last_angle_raw;

	if (diff > 4096) diff -= 8192;
	else if (diff < -4096) diff += 8192;

    // 累積角度に加算 (度数法)
    total_angle += (float)diff * DEG_PER_COUNT;

    // 今回の値を「前回値」として保存
    last_angle_raw = angle_raw;

    last_feedback_time = HAL_GetTick();
}

int16_t RoboMaster::calculate() {
    if (mode == 3) return 0;

    float final_current = 0;
    if (mode == 2) {
    	float target_velocity = pos_pid.compute(target, total_angle);
        final_current = vel_pid.compute(target_velocity, current_velocity);
    }
    else if (mode == 1) {
        final_current = vel_pid.compute(target, current_velocity);
    }
    else if (mode == 0) {
    	final_current = std::max(-vel_pid.output_limit, std::min(target, vel_pid.output_limit));
    }
    return (int16_t)final_current;
}
