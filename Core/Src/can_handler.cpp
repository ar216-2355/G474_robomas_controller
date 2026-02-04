#include "can_handler.hpp"
#include "robomas.hpp"
#include "my_LED.hpp" // LEDが必要ならinclude

// グローバル変数の実体定義
RingBuffer<CanFrame, 64> can3_rx_buffer;

// 外部参照
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
extern RoboMaster motors[16];

// 内部関数: 送信処理
static void CAN_Tx_Internal(FDCAN_HandleTypeDef *hfdcan, uint32_t id, uint8_t* data, uint8_t dlc) {
    FDCAN_TxHeaderTypeDef TxHeader;
    TxHeader.Identifier = id;
    TxHeader.IdType = FDCAN_STANDARD_ID;
    TxHeader.TxFrameType = FDCAN_DATA_FRAME;
    TxHeader.DataLength = FDCAN_DLC_BYTES_8; // シンプル化のため常に8byte枠確保推奨
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
    TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
    TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker = 0;

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, data);
}

// 初期化設定
static void CAN_Config_Instance(FDCAN_HandleTypeDef *hfdcan) {
    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000; // 全受信

    HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig);
    HAL_FDCAN_Start(hfdcan);
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
}

void CAN_Init() {
    CAN_Config_Instance(&hfdcan1);
    CAN_Config_Instance(&hfdcan2);
    CAN_Config_Instance(&hfdcan3);
}

void CAN3_Transmit_Raw(uint32_t id, uint8_t* data, uint8_t dlc) {
    CAN_Tx_Internal(&hfdcan3, id, data, dlc);
}

// ---------------------------------------------------------
// 公開関数: モーター指令の一括送信 (Timer割り込みから呼ぶ)
// ---------------------------------------------------------
void Send_All_Motor_Commands() {
    uint8_t tx_data[8];

    // --- FDCAN1: Motors 0-3 (ID 1-4) -> Send to 0x200 ---
    for (int i=0; i<4; i++) {
        int16_t cmd = motors[i].calculate(); // PID計算
        tx_data[i*2]   = (cmd >> 8) & 0xFF;
        tx_data[i*2+1] = cmd & 0xFF;
    }
    CAN_Tx_Internal(&hfdcan1, 0x200, tx_data, 8);

    // --- FDCAN1: Motors 4-7 (ID 5-8) -> Send to 0x1FF ---
    for (int i=0; i<4; i++) {
        int16_t cmd = motors[4+i].calculate();
        tx_data[i*2]   = (cmd >> 8) & 0xFF;
        tx_data[i*2+1] = cmd & 0xFF;
    }
    CAN_Tx_Internal(&hfdcan1, 0x1FF, tx_data, 8);

    // --- FDCAN2: Motors 8-11 (ID 1-4) -> Send to 0x200 ---
    for (int i=0; i<4; i++) {
        int16_t cmd = motors[8+i].calculate();
        tx_data[i*2]   = (cmd >> 8) & 0xFF;
        tx_data[i*2+1] = cmd & 0xFF;
    }
    CAN_Tx_Internal(&hfdcan2, 0x200, tx_data, 8);

    // --- FDCAN2: Motors 12-15 (ID 5-8) -> Send to 0x1FF ---
    for (int i=0; i<4; i++) {
        int16_t cmd = motors[12+i].calculate();
        tx_data[i*2]   = (cmd >> 8) & 0xFF;
        tx_data[i*2+1] = cmd & 0xFF;
    }
    CAN_Tx_Internal(&hfdcan2, 0x1FF, tx_data, 8);
}


// 割り込みハンドラ
extern "C" void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0) return;

    FDCAN_RxHeaderTypeDef RxHeader;
    uint8_t RxData[8];

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK) return;

    // FDCAN1: Motors 1-8
    if (hfdcan->Instance == FDCAN1) {
    	CAN1_LED();
        if (RxHeader.Identifier >= 0x201 && RxHeader.Identifier <= 0x208) {
            motors[RxHeader.Identifier - 0x201].updateFeedback(RxData);
        }
    }
    // FDCAN2: Motors 9-16
    else if (hfdcan->Instance == FDCAN2) {
    	CAN2_LED();
        if (RxHeader.Identifier >= 0x201 && RxHeader.Identifier <= 0x208) {
            motors[(RxHeader.Identifier - 0x201) + 8].updateFeedback(RxData);
        }
    }
    // FDCAN3: Sniffer
    else if (hfdcan->Instance == FDCAN3) {
    	CAN3_LED();
        CanFrame frame;
        frame.id = RxHeader.Identifier;
        frame.dlc = (RxHeader.DataLength >> 16); // 簡易変換
        for(int i=0; i<8; i++) frame.data[i] = RxData[i];

        // ★リングバッファへ追加
        can3_rx_buffer.push(frame);
    }
}
