#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


// --------------------------------------------------
// 1. 各コマンド用の中身（ペイロード）の定義
// --------------------------------------------------

// [A] 全モーター駆動用 (頻繁に送る: 1msごと)
// ModeとTargetは必ずペアで扱います。
typedef struct {
    uint8_t mode;       // 0:電流, 1:速度, 2:位置, 3:無効(無視)
    float   target;     // 目標値
} __attribute__((packed)) MotorUnit;

// [B] PIDゲイン設定用 (調整時に送る)
// 全台一括だとデータが大きすぎるため、1パケットで「1台分」の設定を送ります。
typedef struct {
    uint8_t motor_id;   // 設定するモーター番号 (1-16)
    float   speed_kp;
    float   speed_ki;
    float   speed_kd;
    float   speed_i_limit;
    float   speed_output_limit;  // 出力電流最大値
    float   position_kp;
    float   position_ki;
    float   position_kd;
    float   position_i_limit;
    float   position_output_limit;  // 出力速度最大値
} __attribute__((packed)) PIDConfig;

// [C] 任意のCANフレーム送信要求 (PCから任意のデータを送りたい時)
typedef struct {
    uint32_t can_id;    // CAN ID (標準/拡張)
    uint8_t  dlc;       // データ長 (0-8)
    uint8_t  data[8];   // データ本体
} __attribute__((packed)) CantTxRequest;


// --------------------------------------------------
// 2. USB送受信用のメインパケット (全体定義)
// --------------------------------------------------

typedef struct {
    // [ヘッダー部]
    uint16_t header;      // 0xA5A5 (パケットの開始識別用)
    uint8_t  command_id;  // パケットの種類 (enum CommandID 参照)

    // [ペイロード部 (共用体)]
    // メモリ領域を共有します。サイズは一番大きい「drive (80byte)」に合わせられます。
    union {
    	// ID: 0x00 -> 緊急停止 (データ不要だがメモリ確保のため)
		uint8_t       dummy[80];

        // ID: 0x01 -> ロボマス16台を一括制御
        MotorUnit     drive[16];

        // ID: 0x02 -> 特定の1台のPIDを変更
        PIDConfig     pid;

        // ID: 0x03 -> 任意のCANフレームを送信
        CantTxRequest can_tx;

    } payload;

    // [フッター部]
    uint16_t checksum;    // データの整合性チェック用

} __attribute__((packed)) USBCtrlPacket;


// コマンドIDの定義
enum CommandID {
    CMD_EMERGENCY_STOP = 0x00, // 緊急停止
    CMD_DRIVE_ALL      = 0x01, // 16台制御
    CMD_SET_PID        = 0x02, // PID設定
    CMD_SEND_CAN       = 0x03  // CAN送信
};



///////////////////////////////// PC → マイコン ///////////////////////////////////

///////////////////////////////// マイコン → PC ///////////////////////////////////



// ロボマス1台分のフィードバック情報 (11 byte)
typedef struct {
    float   angle;        // 累積角度 (度数法)
    float   velocity;     // 現在速度 (rpm)
    int16_t torque;       // 現在トルク電流 (生値)
    uint8_t temp;         // 温度 (℃)
} __attribute__((packed)) MotorFeedbackUnit;

// ---------------------------------------------
// パケットA: ロボマス状態 (定期送信)
// ヘッダー: 0x5A5A
// ---------------------------------------------
typedef struct {
    uint16_t header;      // 0x5A5A
    MotorFeedbackUnit motors[16];
    uint16_t checksum;
} __attribute__((packed)) USBFeedbackPacket;

// ---------------------------------------------
// パケットB: 任意CAN転送 (イベント送信)
// ヘッダー: 0xCACA (Can Can)
// ---------------------------------------------
typedef struct {
    uint16_t header;      // 0xCACA (識別用)
    uint32_t can_id;      // CAN ID
    uint8_t  dlc;         // データ長
    uint8_t  data[8];     // データ本体
    uint16_t checksum;
} __attribute__((packed)) USBCanForwardPacket;



void Prepare_Motor_Packet(USBFeedbackPacket* pkt);



#ifdef __cplusplus
}
#endif
