#pragma once

class PIDController {
public:
    // パラメータ
    float kp = 0, ki = 0, kd = 0;
    float i_limit = 0;      // 積分項の最大値
    float output_limit = 0; // 出力の最大値

    // 内部状態
    float error_sum = 0;
    float last_error = 0;

    // 関数（メソッド）の宣言
    float compute(float target, float current);
    void reset();
};
