#include "app.hpp"
#include "main.h"
#include "usbd_cdc_if.h"
#include "robomas.hpp"
#include "usb_packet.hpp"
#include "can_handler.hpp"
#include "usb_handler.hpp"
#include "my_LED.hpp"
#include "switch.hpp"
#include <cstring> // memcpy用

// ============================================================
// グローバル変数・インスタンス定義
// ============================================================

// ロボマスモーター管理インスタンス
RoboMaster motors[16];

// 現在のシステム状態 (初期値: 準備モード)
SystemState CurrentSystemState = STATE_READY;

// PID設定完了フラグ (ビットマスク: bit0=Motor0, bit15=Motor15)
// 全ビットが1 (0xFFFF) になるまで駆動を開始しない
uint16_t pid_configured_mask = 0;

// PCからの緊急停止要求フラグ
bool is_pc_emergency_req = false;

// USB受信データ (usbd_cdc_if.c で実体定義)
extern USBCtrlPacket USB_Next_Data;
extern volatile uint8_t is_new_data_ready;

// ============================================================
// アプリケーション初期化
// ============================================================
void App_Init() {
    CAN_Init();
    Green_LED(0);
}

// ============================================================
// メインループ (main.cのwhile内で呼ばれる)
// ============================================================
void App_Loop() {
    // --- 静的変数 (状態保持用) ---
    static bool is_system_running = false;       // 内部的な駆動許可フラグ
    static bool last_system_running = false;     // エッジ検出用
    static uint32_t last_motor_tx_time = 0;      // USB送信タイマー
    static uint32_t last_control_time = 0;       // 制御ループタイマー
    static bool last_pb10_state = GPIO_PIN_SET;  // ボタン状態保持

    uint32_t now = HAL_GetTick();

    // ------------------------------------------------------------
    // 1. 入力取得 & 状態遷移判定
    // ------------------------------------------------------------
    bool sw_emergency_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
    bool sw_start_btn_state   = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));

    // [停止条件]
    // 1. PCからの緊急停止要求
    // 2. スタートボタン(PB10)が離された (High) ※デッドマンスイッチ動作
    // 3. 物理非常停止ボタンが押された
    bool sw_stop_cond_pb10 = (sw_start_btn_state == GPIO_PIN_SET);

    if (is_pc_emergency_req || sw_stop_cond_pb10 || sw_emergency_pressed) {
        is_system_running = false;
    }

    // [開始条件]
    // 1. 非常停止要求がない
    // 2. 全16台のPID設定が完了している (0xFFFF)
    // 3. スタートボタンが押された瞬間 (立ち下がりエッジ)
    bool is_all_configured = (pid_configured_mask == 0xFFFF);
    bool is_pb10_falling_edge = (last_pb10_state == GPIO_PIN_SET) && (sw_start_btn_state == GPIO_PIN_RESET);

    if (!is_pc_emergency_req && !sw_emergency_pressed && is_all_configured && is_pb10_falling_edge) {
        is_system_running = true;
    }

    last_pb10_state = sw_start_btn_state;

    // ------------------------------------------------------------
    // 2. CurrentSystemState の確定 (PC通知用)
    // ------------------------------------------------------------
    if (sw_emergency_pressed || is_pc_emergency_req) {
        CurrentSystemState = STATE_EMERGENCY; // 0: 非常停止
    } else if (is_system_running) {
        CurrentSystemState = STATE_DRIVE;     // 2: 駆動
    } else {
        CurrentSystemState = STATE_READY;     // 1: 準備
    }

    // ------------------------------------------------------------
    // 3. 状態変化時のワンショット処理 (エッジ処理)
    // ------------------------------------------------------------

    // A. [停止 -> 駆動] 開始時
    if (is_system_running && !last_system_running) {
        for (int i = 0; i < 16; i++) {
            // I項リセット & バンプレス転送 (急発進防止)
            motors[i].vel_pid.reset();
            motors[i].pos_pid.reset();

            if (motors[i].mode == 2) {
                motors[i].target = motors[i].total_angle;
            } else {
                motors[i].target = 0;
            }
        }
    }

    // B. [駆動 -> 停止] 終了時
    else if (!is_system_running && last_system_running) {
        for (int i = 0; i < 16; i++) {
            motors[i].mode = 0;   // 電流制御へ
            motors[i].target = 0; // 電流0 (脱力)
        }
        Send_All_Motor_Commands(); // 即座に停止信号を送信
    }

    last_system_running = is_system_running;

    // ------------------------------------------------------------
    // 4. 制御周期 (PID計算 & CAN送信) - 2ms周期
    // ------------------------------------------------------------
    if (now - last_control_time >= 2) {
        // 駆動モードのときだけPID計算してCAN送信
        if (CurrentSystemState == STATE_DRIVE) {
            Send_All_Motor_Commands();
        }
        last_control_time = now;
    }

    // ------------------------------------------------------------
    // 5. USBフィードバック (常時送信) - 10ms周期
    // ------------------------------------------------------------
    if (now - last_motor_tx_time >= 10) {
        Update_Mode_LEDs();

        USBFeedbackPacket fb_pkt;
        // CurrentSystemState と pid_configured_mask がパケットに入ります
        Prepare_Motor_Packet(&fb_pkt);

        if (CDC_Transmit_FS((uint8_t*)&fb_pkt, sizeof(USBFeedbackPacket)) == USBD_OK) {
            last_motor_tx_time = now;
        }
    }

    // ------------------------------------------------------------
    // 6. 受信処理 (CAN & USB)
    // ------------------------------------------------------------

    // A. CAN受信中継 (PCへ転送)
    CanFrame frame;
    if (can3_rx_buffer.pop(frame)) {
        USBCanForwardPacket tx_pkt;
        tx_pkt.header = 0xCACA;
        tx_pkt.can_id = frame.id;
        tx_pkt.dlc    = frame.dlc;
        memcpy(tx_pkt.data, frame.data, 8);
        tx_pkt.checksum = 0;
        CDC_Transmit_FS((uint8_t*)&tx_pkt, sizeof(USBCanForwardPacket));
    }

    // B. USBコマンド受信 (PCからの指令)
    if (is_new_data_ready) {
        __disable_irq();
        USBCtrlPacket temp = USB_Next_Data;
        is_new_data_ready = 0;
        __enable_irq();

        // コマンド種別ごとのフラグ更新処理
        if (temp.command_id == CMD_SET_PID) {
            uint8_t id = temp.payload.pid.motor_id;
            if (id < 16) {
                // 設定完了ビットを立てる (ハンドシェイク用)
                pid_configured_mask |= (1 << id);
            }
        }
        else if (temp.command_id == CMD_EMERGENCY_STOP) {
            is_pc_emergency_req = true;  // 緊急停止 ON
        }
        else if (temp.command_id == CMD_RESET_EMERGENCY) {
            is_pc_emergency_req = false; // 緊急停止 OFF (解除)
        }

        // コマンド解析実行
        // 非常停止中であっても、CAN送信(CMD_SEND_CAN)やPID設定(CMD_SET_PID)は
        // 受け付ける仕様にするため、条件分岐なしで実行する。
        // ※駆動コマンド(CMD_DRIVE_ALL)が来ても、上記ステートマシンで
        //   STATE_DRIVEにならなければ無視されるため安全。
        Parse_USB_Packet(&temp);
    }
}
