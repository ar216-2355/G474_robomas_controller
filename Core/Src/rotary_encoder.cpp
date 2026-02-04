#include "rotary_encoder.hpp"

volatile int32_t encoder_count = 0;
volatile uint8_t mode_switch_status = 0;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_0) {
		// --- 時間によるチャタリング除去 ---
		static uint32_t last_time = 0;
		uint32_t current_time = HAL_GetTick(); // 1ms単位の簡易判定

		// 前回のカウントから1ms経過していなければ無視
		// (高速回転する場合はこの値を 0 にするか、マイクロ秒単位の判定が必要)
		if (current_time - last_time < 1) {
			// return; // 必要に応じてコメントアウトを外す
		}
		last_time = current_time;

		// --- 状態比較によるチャタリング除去 ---
		GPIO_PinState a_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);
		GPIO_PinState b_state = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1);

		static GPIO_PinState last_a_state = GPIO_PIN_RESET;
		if (a_state == last_a_state) return; // 状態が変わっていなければノイズとみなす
		last_a_state = a_state;

		// --- 本来のカウント処理 ---
		if (a_state == GPIO_PIN_SET) {
			if (b_state == GPIO_PIN_RESET) encoder_count++;
			else                          encoder_count--;
		} else {
			if (b_state == GPIO_PIN_SET)   encoder_count++;
			else                          encoder_count--;
		}
	}

    if (GPIO_Pin == GPIO_PIN_3) // PA2をスイッチとした場合
	{
		// チャタリングが収まるのを待つ
		for(volatile int i=0; i<500; i++);

		// 今のピンの状態を読み取って変数に格納
		// Pull-upの場合、RESET(0V)の時にスイッチがON(1)と判定
		if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3) == GPIO_PIN_RESET) {
			mode_switch_status = 1;
			HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_SET);
		} else {
			mode_switch_status = 0;
			HAL_GPIO_WritePin(Green_LED_GPIO_Port, Green_LED_Pin, GPIO_PIN_RESET);
		}
	}
}
