#pragma once

#include "main.h" // HALの定義を読み込む

extern "C" {

extern volatile int32_t encoder_count;
extern volatile uint8_t mode_switch_status;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);

}
