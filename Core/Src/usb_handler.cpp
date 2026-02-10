#include "usb_handler.hpp"
#include "app.hpp"         // ★重要: enum SystemState の定義を知るために必要
#include "robomas.hpp"
#include "can_handler.hpp"
#include "my_LED.hpp"
#include <cstring>         // memset用

// =========================================================
// 1. 変数の実体定義 (Memory Allocation)
// =========================================================
// app.cpp で "extern" されている変数の「実体」をここに作ります。
// これがないとリンクエラーになります。
extern "C" {
    extern USBCtrlPacket USB_Next_Data;
    extern volatile uint8_t is_new_data_ready;
}

// =========================================================
// 2. 外部変数の参照 (External References)
// =========================================================
// 他のファイルにある変数を使いに行きます。
extern RoboMaster motors[16];           // main.cpp (or app.cpp)
extern SystemState CurrentSystemState;  // ★app.cpp で計算された現在の状態
extern uint16_t pid_configured_mask; // app.cppの変数を参照

// =========================================================
// 3. 内部ヘルパー関数
// =========================================================

static bool VerifyChecksum(const USBCtrlPacket* packet) {
    uint16_t sum = 0;
    const uint8_t* p = (const uint8_t*)packet;
    for (size_t i = 0; i < sizeof(USBCtrlPacket) - 2; i++) sum += p[i];
    return (sum == packet->checksum);
}

static uint16_t CalcChecksum(const void* packet, size_t size) {
    uint16_t sum = 0;
    const uint8_t* p = (const uint8_t*)packet;
    // checksumは最後の2バイトにある前提
    for (size_t i = 0; i < size - 2; i++) {
        sum += p[i];
    }
    return sum;
}

// =========================================================
// 4. パケット解析関数 (受信処理)
// =========================================================
void Parse_USB_Packet(const USBCtrlPacket* pkt) {
    // 1. ヘッダー確認
    if (pkt->header != 0xA5A5) return;

    // 2. チェックサム確認
    if (!VerifyChecksum(pkt)) return;

    // 3. コマンドIDごとの処理
    switch (pkt->command_id) {

        case CMD_DRIVE_ALL:
            // 駆動モード(STATE_DRIVE)でない時にモーター値を更新しても良いか？
            // -> 安全のため、更新自体はOKだが、app.cpp側で出力がカットされる設計なので
            //    ここでは単純に代入するだけでOKです。
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
                // main.h や can_handler.hpp で宣言されている hfdcan3 を使う
                // （もし見えなければ extern FDCAN_HandleTypeDef hfdcan3; を追加）
                extern FDCAN_HandleTypeDef hfdcan3;

                // 新しく作った関数を呼ぶだけ！
                // HALの複雑な設定は全部向こうでやってくれます
                CAN_Transmit_Safe(
                    &hfdcan3,
                    pkt->payload.can_tx.can_id,
                    (uint8_t*)pkt->payload.can_tx.data,
                    pkt->payload.can_tx.dlc
                );
            }
            break;

        case CMD_EMERGENCY_STOP:
            // ここでの処理も重要ですが、app.cpp側で `is_pc_emergency_req` フラグを
            // 立てて制御しているので、ここでは念のためのパラメータリセットを行います
            for (int i = 0; i < 16; i++) {
                motors[i].mode = 3;
                motors[i].target = 0;
            }
            break;
    }
}

// =========================================================
// 5. モーターパケット作成関数 (送信処理)
// =========================================================
void Prepare_Motor_Packet(USBFeedbackPacket* pkt) {
	memset(pkt, 0, sizeof(USBFeedbackPacket));

	pkt->header = 0x5A5A;
	pkt->system_state = (uint8_t)CurrentSystemState;

	// ★現在の設定状況をPCへ通知
	pkt->pid_configured_mask = pid_configured_mask;

	pkt->reserved = 0;

    // 4. 16台分のデータを詰め込む
    for (int i = 0; i < 16; i++) {
        pkt->motors[i].angle    = motors[i].total_angle;
        pkt->motors[i].velocity = motors[i].current_velocity;
        pkt->motors[i].torque   = motors[i].torque_current_raw;
        pkt->motors[i].temp     = motors[i].temperature;
    }

    // 5. チェックサム計算
    pkt->checksum = CalcChecksum(pkt, sizeof(USBFeedbackPacket));
}
