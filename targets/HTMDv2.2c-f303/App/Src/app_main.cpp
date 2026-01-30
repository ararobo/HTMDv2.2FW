#include "main.h"
#include "common/core/app.hpp"
#include "common/config_defs.hpp"
#include "stm32_gate_driver.hpp"
#include "stm32_encoder.hpp"
#include "stm32_indicator.hpp"
#include "driver_stm32_can.hpp"

// Global handles defined in main.c
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern CAN_HandleTypeDef hcan;

// C entry point wrapper
extern "C" void app_main();

void app_main() {
    // 1. ハードウェアドライバのインスタンス化
    static targets::f303::Stm32GateDriver gate_driver(&htim2);
    static targets::f303::Stm32Encoder encoder(&htim1, common::config::DEFAULT_ENCODER_CPR);
    static targets::f303::Stm32Indicator indicator;
    static gn10_can::drivers::DriverSTM32CAN can_driver(&hcan);

    // 2. DIPスイッチからID読み取り (簡易版)
    // GPIOはmain.cで初期化済み
    uint8_t device_id = 0;
    if (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin) == GPIO_PIN_SET) device_id |= 1 << 0;
    if (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin) == GPIO_PIN_SET) device_id |= 1 << 1;
    if (HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin) == GPIO_PIN_SET) device_id |= 1 << 2;
    if (HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin) == GPIO_PIN_SET) device_id |= 1 << 3;

    // 3. アプリケーションの構築
    static common::core::App app(gate_driver, encoder, indicator, can_driver, device_id);

    // 4. 初期化
    app.init();

    // 5. メインループ
    while (true) {
        app.update();
        
        // 簡易的なウェイト (1ms)
        // 実用的にはタイマー割り込みでフラグを立てて、ここで処理するのが良いが、
        // 今回はHAL_Delayを使用する。
        // 制御周期ウェイト
        // 1000ms / 1000Hz = 1ms
        HAL_Delay(static_cast<uint32_t>(1000.0f / common::config::CONTROL_LOOP_FREQ_HZ));
    }
}
