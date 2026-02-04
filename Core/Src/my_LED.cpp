#include "my_LED.hpp"

static uint16_t keep_pattern = 0b0;

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
