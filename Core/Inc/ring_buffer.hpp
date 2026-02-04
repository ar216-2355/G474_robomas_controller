/**
 * @file ring_buffer.hpp
 * @brief 割り込み安全な簡易リングバッファ (Single Producer - Single Consumer)
 */
#pragma once

#include <stdint.h>
#include <vector>

// テンプレートクラス: Tは保存する型、Sizeはバッファサイズ
template <typename T, uint32_t Size>
class RingBuffer {
private:
    T buffer[Size];
    volatile uint32_t head; // 書き込み位置
    volatile uint32_t tail; // 読み込み位置

public:
    RingBuffer() : head(0), tail(0) {}

    // バッファにデータを追加する (ISRから呼ぶ用)
    // 成功したら true, 満杯なら false
    bool push(const T& item) {
        uint32_t next_head = (head + 1) % Size;

        if (next_head == tail) {
            return false; // バッファ満杯
        }

        buffer[head] = item;
        head = next_head;
        return true;
    }

    // バッファからデータを取り出す (メインループから呼ぶ用)
    // データがあれば item に格納して true, 空なら false
    bool pop(T& item) {
        if (head == tail) {
            return false; // バッファ空っぽ
        }

        item = buffer[tail];
        tail = (tail + 1) % Size;
        return true;
    }

    // 空かどうか
    bool isEmpty() const {
        return head == tail;
    }

    // 満杯かどうか
    bool isFull() const {
        return ((head + 1) % Size) == tail;
    }
};
