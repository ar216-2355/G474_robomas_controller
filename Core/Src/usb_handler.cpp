#include "usb_handler.hpp"
#include "robomas.hpp"
#include "my_LED.hpp"
#include <cstring> // ← ★これを追加してください（memset用）

// main.cpp などで実体定義されている motors を参照する
extern RoboMaster motors[16];

// ヘルパー関数
static bool VerifyChecksum(const USBCtrlPacket* packet) {
    uint16_t sum = 0;
    const uint8_t* p = (const uint8_t*)packet;
    for (size_t i = 0; i < sizeof(USBCtrlPacket) - 2; i++) sum += p[i];
    return (sum == packet->checksum);
}

// パケット解析の本体
void Parse_USB_Packet(const USBCtrlPacket* pkt) {
    // 1. ヘッダー確認
    if (pkt->header != 0xA5A5) return;

    // 2. チェックサム確認
    if (!VerifyChecksum(pkt)) return;

    // 3. コマンドIDごとの処理
    switch (pkt->command_id) {

        case CMD_DRIVE_ALL:
            for (int i = 0; i < 16; i++) {
                if (pkt->payload.drive[i].mode != 3) {
                    motors[i].mode   = pkt->payload.drive[i].mode;
                    motors[i].target = pkt->payload.drive[i].target;
                }

            }
            break;

        case CMD_SET_PID:
            {
                uint8_t id = pkt->payload.pid.motor_id;
                if (id < 16) {
                    motors[id].vel_pid.kp           = pkt->payload.pid.speed_kp;
                    motors[id].vel_pid.ki           = pkt->payload.pid.speed_ki;
                    motors[id].vel_pid.kd           = pkt->payload.pid.speed_kd;
                    motors[id].vel_pid.i_limit      = pkt->payload.pid.speed_i_limit;
                    motors[id].vel_pid.output_limit = pkt->payload.pid.speed_output_limit;

                    motors[id].pos_pid.kp           = pkt->payload.pid.position_kp;
                    motors[id].pos_pid.ki           = pkt->payload.pid.position_ki;
                    motors[id].pos_pid.kd           = pkt->payload.pid.position_kd;
                    motors[id].pos_pid.i_limit      = pkt->payload.pid.position_i_limit;
                    motors[id].pos_pid.output_limit = pkt->payload.pid.position_output_limit;

                    motors[id].vel_pid.reset();
                    motors[id].pos_pid.reset();
                }
            }
            break;

        case CMD_SEND_CAN:
            {
                // ここでCAN送信関数を呼び出す場合は、
                // main.h や CANライブラリを include してください
            }
            break;

        case CMD_EMERGENCY_STOP:
            for (int i = 0; i < 16; i++) {
                motors[i].mode = 3;
                motors[i].target = 0;
            }
            break;
    }
}

// ---------------------------------------------------------
// 内部関数: チェックサム計算 (作成用)
// ---------------------------------------------------------
// パケットの先頭から「checksumフィールドの直前」までのバイト和を計算します
static uint16_t CalcChecksum(const void* packet, size_t size) {
    uint16_t sum = 0;
    const uint8_t* p = (const uint8_t*)packet;

    // checksumは最後の2バイトにある前提なので、size - 2 までループ
    for (size_t i = 0; i < size - 2; i++) {
        sum += p[i];
    }
    return sum;
}

// ---------------------------------------------------------
// 公開関数: モーターパケットの作成 (梱包)
// ---------------------------------------------------------
void Prepare_Motor_Packet(USBFeedbackPacket* pkt) {
    // 1. バッファを念のためゼロクリア (ゴミデータ混入防止)
    //    必須ではありませんが、安全のため推奨
    memset(pkt, 0, sizeof(USBFeedbackPacket));

    // 2. ヘッダー設定
    pkt->header = 0x5A5A;

    // 3. 16台分のデータを詰め込む
    for (int i = 0; i < 16; i++) {
        // グローバルの motors 配列から値を取得
        pkt->motors[i].angle    = motors[i].total_angle;      // 累積角度
        pkt->motors[i].velocity = motors[i].current_velocity; // 速度
        pkt->motors[i].torque   = motors[i].torque_current_raw; // トルク電流
        pkt->motors[i].temp     = motors[i].temperature;      // 温度
    }

    // 4. チェックサムを計算して封をする
    //    構造体全体のサイズを渡して計算させ、最後のメンバに代入
    pkt->checksum = CalcChecksum(pkt, sizeof(USBFeedbackPacket));
}
