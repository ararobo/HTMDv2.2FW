#include "a3921.hpp"
#include "tim.h"
#include "gpio.h"
#include <algorithm>

GateDriver::GateDriver(uint16_t max_duty)
{
    this->max_duty = max_duty;
}

void GateDriver::hardware_init()
{
    // PWMの初期化
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

    HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET); // モーター回転方向
    HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET);       // ブレーキ有効
}

void GateDriver::output(int16_t output)
{
    // 出力値を制限
    output = std::clamp(output, int16_t(-max_duty), int16_t(max_duty));

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
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, output);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 3200);
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