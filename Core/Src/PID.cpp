#include "PID.hpp"
#include <algorithm> // std::clamp用

/**
 * @brief PID計算メイン処理
 * @param target 目標値
 * @param current 現在値
 * @return 制限（output_limit）でクランプされた出力値
 */

float PIDController::compute(float target, float current) {
	const float dt = 0.002f;

    float error = target - current;

    // P項
    float p_out = kp * error;

    // I項 (アンチワインドアップ機能付き)
    error_sum += error * dt;
    error_sum = std::max(-i_limit, std::min(error_sum, i_limit));
    float i_out = ki * error_sum;

    // D項
    float d_out = kd * (error - last_error) / dt;
    last_error = error;

    // 合計出力を制限値でクランプ
    float total_out = p_out + i_out + d_out;
    return std::max(-output_limit, std::min(total_out, output_limit));
}

/**
 * @brief 内部状態のリセット
 */
void PIDController::reset() {
    error_sum = 0;
    last_error = 0;
}
