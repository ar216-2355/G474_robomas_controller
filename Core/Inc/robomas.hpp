#pragma once
#include <cstdint>
#include "PID.hpp"

class RoboMaster {
public:
    uint8_t id = 0;
    uint8_t mode = 3;
    float target = 0;

    PIDController vel_pid;
    PIDController pos_pid;

    // センサー情報
	float current_velocity = 0.0f; // rpm
	int16_t torque_current_raw = 0; // トルク電流 (生値: -16384 ~ 16384)
	uint8_t temperature = 0;       // ℃

	// 角度関連 (多回転対応)
	uint16_t angle_raw = 0;        // 現在の生値 (0-8191)
	uint16_t last_angle_raw = 0;   // 前回の生値
	float total_angle = 0.0f;      // 累積角度 (度数法: 360度を超えて増え続ける)

	bool is_first_update = true;   // 初回受信フラグ

	uint32_t last_feedback_time = 0;

	// メソッド宣言
	// CANで受信した8バイトのデータを渡すと、全情報を更新する
	void updateFeedback(const uint8_t* can_data);

    int16_t calculate();
};
