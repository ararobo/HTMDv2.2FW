#include "stm32_indicator.hpp"

namespace targets::f303 {

Stm32Indicator::Stm32Indicator() {}

bool Stm32Indicator::init() {
    // GPIOはmain.cのMX_GPIO_Initで初期化されている前提
    // ここでは初期状態（全消灯）にする
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
    return true;
}

void Stm32Indicator::set(common::interfaces::IndicatorId id, bool enabled) {
    GPIO_TypeDef* port = nullptr;
    uint16_t pin = 0;

    switch (id) {
        case common::interfaces::IndicatorId::HEARTBEAT:
            port = LED1_GPIO_Port;
            pin = LED1_Pin;
            break;
        case common::interfaces::IndicatorId::DIRECTION:
            port = LED2_GPIO_Port;
            pin = LED2_Pin;
            break;
        case common::interfaces::IndicatorId::ACTIVITY:
            port = LED3_GPIO_Port;
            pin = LED3_Pin;
            break;
        case common::interfaces::IndicatorId::POWER:
        default:
            port = LED4_GPIO_Port;
            pin = LED4_Pin;
            break;
    }

    if (port) {
        HAL_GPIO_WritePin(port, pin, enabled ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

}  // namespace targets::f303
