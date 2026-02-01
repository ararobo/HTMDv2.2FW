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
extern TIM_HandleTypeDef htim6;
extern CAN_HandleTypeDef hcan;

// Timer Interrupt Flag
static volatile bool g_tick_flag = false;

// C entry point wrapper
extern "C" void app_main();

// Timer Interrupt Callback
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        g_tick_flag = true;
    }
}

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

    // Config構築
    common::logic::MotorManagerConfig motor_config;
    motor_config.control_loop_dt = common::config::CONTROL_LOOP_DT;
    motor_config.pid_config = {
        common::config::DefaultPID::Kp,
        common::config::DefaultPID::Ki,
        common::config::DefaultPID::Kd,
        common::config::DefaultPID::IntegralLimit,
        common::config::DefaultPID::OutputLimit
    };
    motor_config.default_accel_limit = common::config::DEFAULT_ACCEL_LIMIT;

    // 3. アプリケーションの構築
    static common::core::App app(gate_driver, encoder, indicator, can_driver, motor_config, device_id);

    // 4. 初期化
    app.init();

    // 制御周期設定の適用 (TIM6)
    // CubeMX設定でTIM6は 64MHz / (63+1) = 1MHz (1us単位) でカウント動作するよう設定されているため、
    // 1MHz を Configの周波数で割ることで、必要なカウント数(ARR)を算出する。
    // 例: 1000Hz -> 1,000,000 / 1000 - 1 = 999
    const uint32_t timer_base_clock = 1000000;
    uint32_t arr_value = (timer_base_clock / static_cast<uint32_t>(common::config::CONTROL_LOOP_FREQ_HZ)) - 1;
    __HAL_TIM_SET_AUTORELOAD(&htim6, arr_value);

    // タイマー割り込み開始
    HAL_TIM_Base_Start_IT(&htim6);

    // 5. メインループ
    while (true) {
        // タイマー割り込み待ち (正確な周期実行のため)
        if (g_tick_flag) {
            g_tick_flag = false;
            app.update();
        }
    }
}
