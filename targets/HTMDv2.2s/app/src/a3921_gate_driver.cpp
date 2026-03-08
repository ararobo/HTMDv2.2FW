/**
 * @file a3921_gate_driver.cpp
 * @author Gento Aiba (aiba-gento)
 * @brief A3921ゲートドライバの具象クラス
 * @version 0.2
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "app/a3921_gate_driver.hpp"

#include <algorithm>
#include <cstdint>

#include "gpio.h"
#include "tim.h"

A3921GateDriver::A3921GateDriver(uint16_t max_duty) : max_duty_(max_duty) {}

void A3921GateDriver::hardware_init()
{
    // PWM および GPIO の初期化
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);  // モーター回転方向: 正
    set_brake(true);  // 自クラスの set_brake() 経由で統一する
}

void A3921GateDriver::output(float output)
{
    // 正規化値 [-1.0, 1.0] を PWM カウンタ値 [0, max_duty_] へスケーリング
    output = std::clamp(output, -1.0f, 1.0f) * static_cast<float>(max_duty_);

    // 負値の場合は PHASE ピンで回転方向を切り替え、絶対値を使用する
    if (output < 0.0f) {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_RESET);
        output = -output;
    } else {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);
    }

    // CH1: 可変デューティ, CH2: 常に max_duty_（A3921 の SR 制御用）
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, static_cast<uint32_t>(output));
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, max_duty_);
}

void A3921GateDriver::set_brake(bool brake)
{
    GPIO_PinState state = brake ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, state);
}