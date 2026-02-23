#include "app.hpp"
#include "can_handler.hpp"
#include "main.h"
#include "my_LED.hpp"
#include "robomas.hpp"
#include "switch.hpp"
#include "usb_handler.hpp"
#include "usb_packet.hpp" // PC側と共通のヘッダ (pack(1)対応済み前提)
#include "usbd_cdc_if.h"
#include <cstring> // memcpy

// ============================================================
// グローバル変数
// ============================================================
RoboMaster motors[16];
SystemState CurrentSystemState = STATE_READY;
uint16_t pid_configured_mask = 0; // bit0=Motor1 ... bit15=Motor16
bool is_pc_emergency_req = false;

// USB受信データ (usbd_cdc_if.c等で定義)
extern USBCtrlPacket USB_Next_Data;
extern volatile uint8_t is_new_data_ready;

extern RingBuffer<CanFrame, 64> can3_rx_buffer;

// 内部状態フラグ
static bool is_system_running = false;

void Send_All_Motor_Zero() {
  // 既存の Send_All_Motor_Commands が motors[i].calculate()
  // を使って送信しているなら、 一時的にモードを強制的に「電流制御(0) &
  // 目標0」にして計算させるのが一番安全で確実です。

  // 1. 現在の状態をバックアップ (必要なら)
  // 今回は「停止時」に呼ばれる関数なので、バックアップせずとも
  // モータークラスの状態を「脱力」にしてしまって問題ありません。

  for (int i = 0; i < 16; i++) {
    motors[i].mode = 3;   // 無効モード
    motors[i].target = 0; // 電流 0
  }

  // 2. 通常の送信関数を呼ぶ
  // これにより calculate() が 0 を返し、それがCANで送信されます。
  Send_All_Motor_Commands();
}

// ============================================================
// アプリケーション初期化
// ============================================================
void App_Init() {
  CAN_Init();
  Green_LED(0); // 消灯

  // モーター初期化
  for (int i = 0; i < 16; i++) {
    motors[i].init(i + 1); // ID: 1-16
  }
}

// ============================================================
// メインループ
// ============================================================
void App_Loop() {
  static uint32_t last_motor_tx_time = 0;
  static uint32_t last_control_time = 0;

  static bool prev_sw_start = false;
  static bool is_first_loop = true;

  static uint32_t last_pc_msg_time = HAL_GetTick();

  uint32_t now = HAL_GetTick();

  // ------------------------------------------------------------
  // 1. 入力取得 & 状態遷移判定 (デッドマンスイッチ論理)
  // ------------------------------------------------------------
  // ★修正1: 論理反転（GPIO_PIN_RESET つまり Low のときを ON(true) と判定する）
  bool raw_sw_start = (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_RESET);

  // ★追加:
  // 起動時の安全ロック（マイコン起動時に既にスイッチがONだった場合、一度OFFにするまで無視する）
  if (is_first_loop) {
    prev_sw_start = raw_sw_start;
    is_first_loop = false;
  }

  // EMS (PA7): 押すとHigh (Active High) と仮定
  bool sw_emergency_on = (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_7) == GPIO_PIN_SET);

  bool is_pc_timeout = (now - last_pc_msg_time > 500);

  // [安全条件] 全てクリアで駆動許可
  bool is_safety_ok =
      !sw_emergency_on && !is_pc_emergency_req && !is_pc_timeout;

  // [設定条件] 全PID設定済み
  bool is_config_ok = (pid_configured_mask == 0xFFFF);

  // [状態遷移]
  if (is_safety_ok && is_config_ok && raw_sw_start) {
    // 全条件クリア + ボタン押下中 -> DRIVE
    if (!is_system_running) {
      if (raw_sw_start == true && prev_sw_start == false) {
        // 立ち上がりエッジ (READY -> DRIVE)
        for (int i = 0; i < 16; i++) {
          motors[i].vel_pid.reset();
          motors[i].pos_pid.reset();
          // ★修正: ランプ用リセット (内部で target = current_setpoint = ...
          // が行われる)
          motors[i].resetSetpoint();
        }
        is_system_running = true;
      }
    }
  } else {
    // 何か一つでもダメなら -> STOP (READY or EMERGENCY)
    if (is_system_running) {
      // 立ち下がりエッジ (DRIVE -> READY)
      for (int i = 0; i < 16; i++) {
        motors[i].mode = 0;   // 電流制御
        motors[i].target = 0; // 脱力
      }
      is_system_running = false;
    }
  }

  prev_sw_start = raw_sw_start;

  // ------------------------------------------------------------
  // 2. CurrentSystemState 更新 (PC通知用)
  // ------------------------------------------------------------
  if (!is_safety_ok) {
    CurrentSystemState = STATE_EMERGENCY; // 0
    Green_LED(0);                         // 消灯
  } else if (is_system_running) {
    CurrentSystemState = STATE_DRIVE; // 2
    Green_LED(1);                     // 点灯
  } else {
    CurrentSystemState = STATE_READY; // 1
    // 点滅処理 (簡易)
    if ((now % 1000) < 500)
      Green_LED(1);
    else
      Green_LED(0);
  }

  // ------------------------------------------------------------
  // 3. 制御周期 (PID & CAN) - 1ms or 2ms
  // ------------------------------------------------------------
  if (now - last_control_time >= 2) {
    // 駆動中のみPID計算して送信
    if (is_system_running) {
      // 各モーターの .calculate() を呼んで CAN送信
      Send_All_Motor_Commands();
    } else {
      // 停止中も念のためゼロを送り続けるか、送らないか
      // 安全のため「電流0」を送り続けるのがRoboMasterでは推奨
      // (通信タイムアウトでESCがエラーになるのを防ぐため)
      Send_All_Motor_Zero();
    }
    last_control_time = now;
  }

  // ------------------------------------------------------------
  // 4. USBフィードバック送信 - 10ms
  // ------------------------------------------------------------
  if (now - last_motor_tx_time >= 10) {
    static USBFeedbackPacket fb_pkt;
    memset(&fb_pkt, 0, sizeof(fb_pkt));

    fb_pkt.header = 0x5A5A;
    fb_pkt.system_state = (uint8_t)CurrentSystemState;
    fb_pkt.pid_configured_mask = pid_configured_mask; // ★ハンドシェイク用

    // モーター情報を詰める
    for (int i = 0; i < 16; i++) {
      fb_pkt.motors[i].angle = motors[i].total_angle;
      fb_pkt.motors[i].velocity = motors[i].current_velocity;
      fb_pkt.motors[i].torque = motors[i].current_torque;
      fb_pkt.motors[i].temp = motors[i].temperature;
    }

    // チェックサム計算 (PC側と合わせる)
    uint16_t sum = 0;
    uint8_t *p = (uint8_t *)&fb_pkt;
    for (size_t i = 0; i < sizeof(USBFeedbackPacket) - 2; i++)
      sum += p[i];
    fb_pkt.checksum = sum;

    CDC_Transmit_FS((uint8_t *)&fb_pkt, sizeof(USBFeedbackPacket));

    last_motor_tx_time = now;

    // LED更新 (制御モード表示など)
    Update_Mode_LEDs();
  }

  CanFrame rx_frame;
	// pop() 自体が成功(true)か失敗/空(false)を返すので、これで「空でないかの確認」と「取り出し」を同時に行えます。
	if (can3_rx_buffer.pop(rx_frame)) {
		USBCanForwardPacket forward_pkt;

		forward_pkt.header = 0xCACA; // usb_packet.hpp の定義に合わせる
		forward_pkt.can_id = rx_frame.id;
		forward_pkt.dlc    = rx_frame.dlc;
		memcpy(forward_pkt.data, rx_frame.data, 8);

		// チェックサム計算
		uint16_t sum = 0;
		uint8_t *p = (uint8_t *)&forward_pkt;
		// checksum フィールド（最後の2バイト）を除いたサイズで計算
		for (size_t i = 0; i < sizeof(USBCanForwardPacket) - 2; i++) {
		  sum += p[i];
		}
		forward_pkt.checksum = sum;

		// USB経由でPCへ送信
		CDC_Transmit_FS((uint8_t *)&forward_pkt, sizeof(USBCanForwardPacket));
	}

  // ------------------------------------------------------------
  // 5. USBデータ受信処理
  // ------------------------------------------------------------
  if (is_new_data_ready) {
	USBCtrlPacket pkt;

	// 割り込み禁止でコピー
	__disable_irq();
	memcpy(&pkt, &USB_Next_Data, sizeof(USBCtrlPacket));
	is_new_data_ready = 0;
	__enable_irq();

	// ★★★ 修正: ここから下を追加・変更 ★★★

	// チェックサムの計算
	uint16_t sum = 0;
	uint8_t *p = (uint8_t *)&pkt;
	for (size_t i = 0; i < sizeof(USBCtrlPacket) - 2; i++) {
	  sum += p[i];
	}

	// 1. ヘッダーが 0xA5A5 であること
	// 2. チェックサムが一致していること
	if (pkt.header == 0xA5A5 && pkt.checksum == sum) {

	  // 正しいパケットを受信したときのみタイムアウトを更新
	  last_pc_msg_time = HAL_GetTick();

	  // コマンド処理
	  switch (pkt.command_id) {
	  case CMD_SET_PID: {
		uint8_t id = pkt.payload.pid.motor_id; // 1-16
		if (id >= 1 && id <= 16) {
		  // PID設定反映
		  motors[id - 1].setPID(pkt.payload.pid);
		  // マスクビットを立てる
		  pid_configured_mask |= (1 << (id - 1));
		}
		break;
	  }
	  case CMD_DRIVE_ALL:
		// DRIVEモード中のみ、目標値を更新
		if (is_system_running) {
		  for (int i = 0; i < 16; i++) {
			motors[i].mode = pkt.payload.drive[i].mode;
			motors[i].updateTarget(pkt.payload.drive[i].target);
		  }
		}
		break;
	  case CMD_EMERGENCY_STOP:
		is_pc_emergency_req = true;
		break;
	  case CMD_RESET_EMERGENCY:
		is_pc_emergency_req = false;
		break;
	  case CMD_SEND_CAN: {
		// 外部定義されているFDCAN3のハンドルを呼び出す
		extern FDCAN_HandleTypeDef hfdcan3;

		// can_handler.cpp にある安全な送信関数を使ってFDCAN3から送信
		CAN_Transmit_Safe(
			&hfdcan3,
			pkt.payload.can_tx.can_id,
			(uint8_t*)pkt.payload.can_tx.data,
			pkt.payload.can_tx.dlc
		);
		break;
	  }
	  }
	}
  }
}
