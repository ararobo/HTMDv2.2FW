#pragma once
#include <stdint.h>

class TrapezoidalController
{
private:
    int16_t prev_out = 0;      // 前回の出力
    uint8_t control_cycle = 5; // 制御周期

public:
    /**
     * @brief 台形制御を行う
     *
     * @param output 出力
     * @param max_acceleration 最大加速度
     * @return int16_t 台形制御の出力
     */
    int16_t trapezoidal_control(int16_t output, uint8_t max_acceleration);

    /**
     * @brief 台形制御の初期化
     *
     */
    void reset_trapezoidal_control();
};