#include "gate_driver.hpp"
#include <algorithm>
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void GateDriver::hardware_init()
{
    max_output = 3199; // 最大出力
    // モータードライバの初期化
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET); // ブレーキ有効
    // エンコーダーの初期化
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

void GateDriver::run(int16_t output, uint16_t max_output)
{
    // 出力を最大出力に制限
    output = std::clamp(int(output), -int(max_output), int(max_output));
    // 出力が負の場合は回転方向を反転
    if (output < 0)
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_RESET);
        output = -output;
    }
    else
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);
    }
    // 出力を設定
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3200);
}

int16_t GateDriver::get_count()
{
    // エンコーダのカウントを取得
    uint16_t enc_buff = TIM1->CNT;
    TIM1->CNT = 0;
    return static_cast<int16_t>(enc_buff); // カウントをint16_tに変換
}

void GateDriver::set_brake(bool brake)
{
    if (brake)
    {
        HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_RESET);
    }
}