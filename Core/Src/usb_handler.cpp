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
            for (int i = 0; i < 16; i++) {
                if (pkt->payload.drive[i].mode != 3) {
                    motors[i].mode   = pkt->payload.drive[i].mode;
                    motors[i].target = pkt->payload.drive[i].target;
                }
            }
            break;

        case CMD_SET_PID:
				{
					// PCは 1〜16 のIDを送ってくる
					uint8_t id_raw = pkt->payload.pid.motor_id;

					// 配列のインデックス(0〜15)に変換
					int idx = id_raw - 1;

					// 範囲チェック (0〜15)
					if (idx >= 0 && idx < 16) {
						// パラメータ適用
						motors[idx].vel_pid.kp           = pkt->payload.pid.speed_kp;
						motors[idx].vel_pid.ki           = pkt->payload.pid.speed_ki;
						motors[idx].vel_pid.kd           = pkt->payload.pid.speed_kd;
						motors[idx].vel_pid.i_limit      = pkt->payload.pid.speed_i_limit;
						motors[idx].vel_pid.output_limit = pkt->payload.pid.speed_output_limit;

						motors[idx].pos_pid.kp           = pkt->payload.pid.position_kp;
						motors[idx].pos_pid.ki           = pkt->payload.pid.position_ki;
						motors[idx].pos_pid.kd           = pkt->payload.pid.position_kd;
						motors[idx].pos_pid.i_limit      = pkt->payload.pid.position_i_limit;
						motors[idx].pos_pid.output_limit = pkt->payload.pid.position_output_limit;

						motors[idx].vel_pid.reset();
						motors[idx].pos_pid.reset();

						// ★★★★★ ここを追加してください！ ★★★★★
						// これがないとマスクが永遠に0のままです
						pid_configured_mask |= (1 << idx);
					}
				}
				break;

        case CMD_SEND_CAN:
            {
                extern FDCAN_HandleTypeDef hfdcan3;
                CAN_Transmit_Safe(
                    &hfdcan3,
                    pkt->payload.can_tx.can_id,
                    (uint8_t*)pkt->payload.can_tx.data,
                    pkt->payload.can_tx.dlc
                );
            }
            break;

        case CMD_EMERGENCY_STOP:
            for (int i = 0; i < 16; i++) {
                motors[i].mode = 3;
                motors[i].target = 0;
            }
            // PCからの停止要求フラグを立てる処理が必要ならここに追加
            // extern bool is_pc_emergency_req;
            // is_pc_emergency_req = true;
            break;

        case CMD_RESET_EMERGENCY: // これも忘れずに！
            // extern bool is_pc_emergency_req;
            // is_pc_emergency_req = false;
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
        pkt->motors[i].torque = motors[i].current_torque;
        pkt->motors[i].temp     = motors[i].temperature;
    }

    // 5. チェックサム計算
    pkt->checksum = CalcChecksum(pkt, sizeof(USBFeedbackPacket));
}
