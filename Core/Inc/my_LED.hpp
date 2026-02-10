#pragma once

#include "main.h" // HALの定義を読み込む

extern "C" {

void updateLEDs(uint16_t pattern) ;

void updateLED(uint8_t num, int8_t state); // num : LEDの番号、　state : 1 -> ON , 0 -> OFF , -1 -> Toggle

void Green_LED(uint8_t a);

void CAN1_LED(void);
void CAN2_LED(void);
void CAN3_LED(void);

void Update_Mode_LEDs();

}
