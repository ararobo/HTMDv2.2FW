#include "stm32_gate_driver.hpp"

#include <algorithm>
#include <cmath>

namespace targets::f303 {

Stm32GateDriver::Stm32GateDriver(TIM_HandleTypeDef* htim_pwm)
    : htim_pwm_(htim_pwm) {}

bool Stm32GateDriver::init() {
    // PWM開始
    if (HAL_TIM_PWM_Start(htim_pwm_, TIM_CHANNEL_1) != HAL_OK) return false;
    if (HAL_TIM_PWM_Start(htim_pwm_, TIM_CHANNEL_2) != HAL_OK) return false;

    // Gate Driver Enable (Active Low DISABLE pin -> High to Disable, Low to Enable)
    // defined as DISABLE_Pin in main.h
    // ここでは初期化直後に制御可能にするため、DisableピンをLow(Active=Enable)にする。
    HAL_GPIO_WritePin(DISABLE_GPIO_Port, DISABLE_Pin, GPIO_PIN_RESET);

    return true;
}

bool Stm32GateDriver::output(float output) {
    if (brake_mode_) {
        // ブレーキ: 両方High または 両方Low (ドライバによる)
        // 一般的なHブリッジはHigh/Highでショートブレーキ、Low/Lowでコースト(予期しないブレーキ)
        // ここではSlow Decay (Short Brake)を意図して Low/Low または High/High
        // 安全のため一旦Low/Low = Coastにする（BrakeModeと言いつつ）
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_2, 0);
        return true;
    }

    // 出力制限
    output = std::clamp(output, -1.0f, 1.0f);

    // Period取得 (ARR)
    uint32_t period = __HAL_TIM_GET_AUTORELOAD(htim_pwm_);
    uint32_t duty = static_cast<uint32_t>(std::abs(output) * period);

    if (output > 0) {
        // 正転: CH1=PWM, CH2=0
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_1, duty);
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_2, 0);
    } else {
        // 逆転: CH1=0, CH2=PWM
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_1, 0);
        __HAL_TIM_SET_COMPARE(htim_pwm_, TIM_CHANNEL_2, duty);
    }

    return true;
}

void Stm32GateDriver::set_brake_mode(bool enabled) {
    brake_mode_ = enabled;
}

}  // namespace targets::f303
