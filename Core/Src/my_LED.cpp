#include "my_LED.hpp"
#include "app.hpp"
#include "robomas.hpp"

static uint16_t keep_pattern = 0b0;

extern RoboMaster motors[16];

extern SystemState CurrentSystemState;

void updateLEDs(uint16_t pattern) {
    for (int i = 0; i < 16; i++) {
        // 1. 最上位ビット(MSB)から順にSER(PC13)へセット
        // 1ならSET、0ならRESET
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, (pattern & (0x8000 >> i)) ? GPIO_PIN_SET : GPIO_PIN_RESET);

        // 2. SRCLK(PC15)にパルスを送ってシフト
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET);
    }

    // 3. 16個送り終えたらRCLK(PC14)を叩いて反映
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

    keep_pattern = pattern;
}

void updateLED(uint8_t num, int8_t state) {
    // numが1〜16の範囲外なら何もしない（安全策）
    if (num >= 17) return;
    if (num == 0) return;

    if (state == 1) {
        // num番目のビットを1にする (SET)
        keep_pattern |= (1 << (num-1));
    } else if (state == 0) {
        // num番目のビットを0にする (RESET)
        keep_pattern &= ~(1 << (num-1));
    } else if (state == -1) {
        // num番目のビットを反転させる (TOGGLE)
        keep_pattern ^= (1 << (num-1));
    }

    // 更新したパターンを実際にLEDへ送信する
    updateLEDs(keep_pattern);
}

void Green_LED(uint8_t a){
	if(a){
		HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
	}else{
		HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
	}
}

void CAN1_LED(void){
	HAL_GPIO_TogglePin(CAN1_LED_GPIO_Port, CAN1_LED_Pin);
}

void CAN2_LED(void){
	HAL_GPIO_TogglePin(CAN2_LED_GPIO_Port, CAN2_LED_Pin);
}

void CAN3_LED(void){
	HAL_GPIO_TogglePin(CAN3_LED_GPIO_Port, CAN3_LED_Pin);
}

// ============================================================
// LED制御関数 (10msごとに呼び出し)
// ============================================================
void Update_Mode_LEDs() {
    uint16_t led_pattern = 0;
    uint32_t now = HAL_GetTick();

    // 1. 各モーターのステータスLED (シフトレジスタ)
    for (int i = 0; i < 16; i++) {
        // フィードバックが途絶えていたら消灯 (Dead)
        if (now - motors[i].last_feedback_time > 200) continue;

        switch (motors[i].mode) {
            case 1: // 速度制御: 常時点灯
                led_pattern |= (1 << i);
                break;
            case 2: // 位置制御: 1秒周期点滅
                if ((now % 1000) < 500) led_pattern |= (1 << i);
                break;
            case 0: // 電流制御: 高速点滅
                if ((now % 200) < 100) led_pattern |= (1 << i);
                break;
            case 3: // 無効: 心拍点滅 (生存確認)
                if ((now % 2000) < 100) led_pattern |= (1 << i);
                break;
            default: break;
        }
    }
    updateLEDs(led_pattern);

    // 2. システム状態用 緑LED (Green_LED)
    switch (CurrentSystemState) {
        case STATE_EMERGENCY:
            // 非常停止: 完全消灯
            Green_LED(0);
            break;

        case STATE_DRIVE:
            // 駆動中: 常時点灯
            Green_LED(1);
            break;

        case STATE_READY:
        default:
            // 準備中: ゆったりとした心拍点滅 (2秒に1回ピカッ)
            if ((now % 2000) < 100) {
                Green_LED(1);
            } else {
                Green_LED(0);
            }
            break;
    }
}
