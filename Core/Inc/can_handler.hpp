#pragma once
#include "main.h" // HALドライバやFDCANハンドルのために必要
#include "fdcan.h"
#include "ring_buffer.hpp"

// リングバッファに保存するデータ型
struct CanFrame {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
};

// 外部からアクセスできるリングバッファの実体
extern RingBuffer<CanFrame, 64> can3_rx_buffer;

// 関数宣言
void CAN_Init();
void Send_All_Motor_Commands(); // 定期的に呼ぶ
void CAN_Transmit_Safe(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data, uint8_t len);
