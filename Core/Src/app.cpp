#include "app.hpp"
#include "main.h"
#include "usbd_cdc_if.h" // USB送信関数を使うため
#include "robomas.hpp"
#include "can_handler.hpp"
#include "usb_handler.hpp"
#include "my_LED.hpp"
#include "switch.hpp"

// --- グローバルインスタンス ---
RoboMaster motors[16]; // ここに実体がある

// USB受信データ (usbd_cdc_if.c から書き込まれる想定)
extern USBCtrlPacket USB_Next_Data;
extern volatile uint8_t is_new_data_ready;

bool last_pb10_state = GPIO_PIN_SET;
bool is_pc_emergency_req = false;

bool is_system_running = false;

void Update_Mode_LEDs() {
    uint16_t led_pattern = 0;
    uint32_t now = HAL_GetTick();

    for (int i = 0; i < 16; i++) {
        // ------------------------------------------------
        // 1. 生存確認 (Dead Check)
        // ------------------------------------------------
        // 200ms以上フィードバックがなければ「死」とみなし消灯
        if (now - motors[i].last_feedback_time > 200) {
            // ビットを立てない (OFF)
            continue;
        }

        // ------------------------------------------------
        // 2. モード別の光り方 (Mode Visualization)
        // ------------------------------------------------
        switch (motors[i].mode) {

            // Mode 1: 速度制御 -> 【常時点灯】
            case 1:
                led_pattern |= (1 << i);
                break;

            // Mode 2: 位置制御 -> 【ゆっくり点滅 (1秒周期)】
            case 2:
                // 500ms ON, 500ms OFF
                if ((now % 1000) < 500) {
                    led_pattern |= (1 << i);
                }
                break;

            // Mode 0: 電流制御 -> 【高速点滅 (0.2秒周期)】
            case 0:
                // 100ms ON, 100ms OFF
                if ((now % 200) < 100) {
                    led_pattern |= (1 << i);
                }
                break;

            // Mode 3: 無効(脱力) -> 【心拍 (2秒に1回ピカッ)】
            case 3:
                // 2000msのうち、最初の100msだけON
                if ((now % 2000) < 100) {
                    led_pattern |= (1 << i);
                }
                break;

            default:
                // 未定義モード -> OFF
                break;
        }
    }

    // シフトレジスタへ送信
    updateLEDs(led_pattern);
}

void App_Init() {
    CAN_Init();
    // モーター初期化などが必要ならここ
    Green_LED(0);
}

void App_Loop() {
	static uint32_t last_motor_tx_time = 0;
	static uint32_t last_control_time = 0;

	// ★追加: 前回のシステム状態を保持する変数 (エッジ検出用)
	static bool last_system_running = false;

	uint32_t now = HAL_GetTick();

	// ============================================================
	// 1. 入力状態の取得 & モード遷移判定
	// ============================================================
	// (ここは前回のままです)
	bool sw_emergency_pressed = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);
	bool sw_start_btn_state   = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10));
	bool sw_stop_cond_pb10 = (sw_start_btn_state == GPIO_PIN_SET);

	if (is_pc_emergency_req || sw_stop_cond_pb10 || sw_emergency_pressed) {
		is_system_running = false;
	}

	bool is_pb10_falling_edge = (last_pb10_state == GPIO_PIN_SET) && (sw_start_btn_state == GPIO_PIN_RESET);
	if (!is_pc_emergency_req && !sw_emergency_pressed && is_pb10_falling_edge) {
		is_system_running = true;
	}
	last_pb10_state = sw_start_btn_state;

	// ============================================================
	// ★追加: 状態変化時のワンショット処理 (ここが重要！)
	// ============================================================

	// A. [停止 -> 駆動] に切り替わった瞬間 (立ち上がりエッジ)
	if (is_system_running && !last_system_running) {
		for (int i = 0; i < 16; i++) {
			// 1. PIDの積分項(I項)などをリセット
			//    これがないと、停止中に溜まったエラーで急発進します
			motors[i].vel_pid.reset();
			motors[i].pos_pid.reset();

			// 2. 目標値を「現在値」に合わせる (バンプレス転送)
			//    PCから目標値が来ていても、開始瞬間は現在位置/速度を目標にすることで
			//    偏差をゼロにし、急激な電流発生を防ぎます。
			if (motors[i].mode == 2) {
				// 位置制御モードなら、現在の角度を目標にする
				motors[i].target = motors[i].total_angle;
			} else {
				// 速度/電流制御なら、目標を0にする (安全策)
				motors[i].target = 0;
			}
		}
	}

	// B. [駆動 -> 停止] に切り替わった瞬間 (立ち下がりエッジ)
	else if (!is_system_running && last_system_running) {
		// 全ロボマスに「電流値0」を即座に送る
		for (int i = 0; i < 16; i++) {
			// モードを一時的に「電流制御(0)」または「無効(3)」にする手もありますが、
			// PID計算結果が0になるようにリセットするのが早いです
			motors[i].mode = 0; // 電流制御モード
			motors[i].target = 0; // 0A
		}
		// 即座にCAN送信を実行 (周期を待たずに送る)
		Send_All_Motor_Commands();
	}

	// 現在の状態を「過去」として保存
	last_system_running = is_system_running;


	// ============================================================
	// 2. モードに応じたLED出力
	// ============================================================
	if (is_system_running) {
		Green_LED(1);
	} else {
		Green_LED(0);
	}

	// ============================================================
	// 3. 制御周期 (PID計算 & CAN送信)
	// ============================================================
	if (now - last_control_time >= 2) {

		if (is_system_running) {
			// 駆動中のみ計算して送信
			Send_All_Motor_Commands();
		}
		// 停止中は送信しない (既に切り替わり瞬間に0を送っているので放置でOK)
		// ※RoboMasterのESCは通信が途絶えると自動で脱力する機能もあります

		last_control_time = now;
	}

	// ... (以下、FDCAN3中継・USB通信などはそのまま) ...
	// ============================================================
	// 4. FDCAN3の中継 (常時動作)
	// ============================================================
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

	// ============================================================
	// 5. ロボマス状態の定期的送信 (常時動作)
	// ============================================================
	else if (now - last_motor_tx_time >= 10) {
		Update_Mode_LEDs();

		USBFeedbackPacket fb_pkt;
		Prepare_Motor_Packet(&fb_pkt);
		if (CDC_Transmit_FS((uint8_t*)&fb_pkt, sizeof(USBFeedbackPacket)) == USBD_OK) {
			last_motor_tx_time = now;
		}
	}

	// ============================================================
	// 6. PCからのコマンド受信処理
	// ============================================================
	if (is_new_data_ready) {
		__disable_irq();
		USBCtrlPacket temp = USB_Next_Data;
		is_new_data_ready = 0;
		__enable_irq();

		if (temp.command_id == CMD_EMERGENCY_STOP) {
			is_pc_emergency_req = true;
		} else {
			is_pc_emergency_req = false;
		}

		Parse_USB_Packet(&temp);
	}
}
