#pragma once

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

// システムの状態定義
// usb_packet.hpp のコメントにある数値と合わせます
typedef enum {
    STATE_EMERGENCY = 0, // 非常停止モード
    STATE_READY     = 1, // 準備モード (入力待ち)
    STATE_DRIVE     = 2  // 駆動モード
} SystemState;

void Send_All_Motor_Zero();

// どちらもmain.cに置かれている
void App_Init(); // 初期化関数
void App_Loop(); // ループ関数


#ifdef __cplusplus
}
#endif
