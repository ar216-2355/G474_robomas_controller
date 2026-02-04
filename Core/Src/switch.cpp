#include "switch.hpp"

/**
 * @brief 74HC165からデータを読み取る
 * PA0 (8pin): Parallel Load
 * PA1 (9pin): Clock Pulse
 * PA2 (10pin): Data Input
 */
uint8_t read_shift_register(void) {
    uint8_t data = 0;

    // 初期状態: ClockをLow、LoadをHigh
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // PA1 (Clock) Low
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);   // PA0 (Load) High
    for(volatile int i=0; i<50; i++);

    // 1. Parallel Load (PA0) を Low にしてデータを取り込み
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    for(volatile int i=0; i<100; i++);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    for(volatile int i=0; i<100; i++);

    // 2. 8ビット分読み取る
    for (int i = 0; i < 8; i++) {
        // PA2 (10番ピン) からデータを読む
        if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2) == GPIO_PIN_SET) {
            data |= (1 << (7 - i));
        }

        // クロック (PA1) を叩く (立ち上がりエッジでシフト)
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
        for(volatile int j=0; j<50; j++);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
        for(volatile int j=0; j<50; j++);
    }

    return data;
}
