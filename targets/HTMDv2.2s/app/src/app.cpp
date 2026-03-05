/**
 * @file app.cpp
 * @author Gento Aiba (aiba-gento)
 * @brief アプリケーションの実装
 * @version 2.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026
 *
 */
#include "app/app.hpp"

#include <cstdint>
#include <optional>

#include "app/a3921_gate_driver.hpp"
#include "app/incremental_encoder.hpp"
#include "drivers/stm32_fdcan/driver_stm32_fdcan.hpp"
#include "fdcan.h"
#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/devices/motor_driver_server.hpp"
#include "gn10_motor/motor_controller.hpp"
#include "gpio.h"
#include "tim.h"

// ---------------------------------------------------------------------------
// ファイルローカル定数
// ---------------------------------------------------------------------------
namespace {

/// エンコーダ1回転あたりのカウント数
constexpr uint16_t ENCODER_MAX_COUNT = 4096U;

/// PWMタイマーのオートリロード値 (htim2.Period)
constexpr uint16_t PWM_MAX_DUTY = 3199U;

/// 制御周期 [s] (htim6: 64MHz / 64 / 1000 = 1kHz → 1ms)
constexpr float CONTROL_DT_S = 0.001f;

/// LED1 点滅間隔 [制御周期] (100 × 1ms = 100ms ごとにトグル)
constexpr uint32_t LED1_BLINK_CYCLES = 100U;

// ---------------------------------------------------------------------------
// App クラス (実装詳細 — 外部に公開しない)
// ---------------------------------------------------------------------------

/**
 * @brief アプリケーション本体
 *
 * 生成順の依存関係:
 *   can_driver_ → can_bus_ → (setup時) can_server_ → motor_
 */
class App
{
public:
    App()
        : can_driver_(&hfdcan1),
          can_bus_(can_driver_),
          gate_driver_(PWM_MAX_DUTY),
          encoder_(ENCODER_MAX_COUNT),
          led1_count_(0)
    {
    }

    /**
     * @brief ハードウェア初期化・制御系起動
     *        main() から1回だけ呼ばれる
     */
    void setup()
    {
        const uint8_t board_id = read_board_id();

        // CAN ドライバ初期化 (フィルタ設定 + 受信割り込み有効)
        can_driver_.init();

        // ゲートドライバ・エンコーダ初期化
        gate_driver_.hardware_init();
        encoder_.hardware_init();

        // 実行時パラメータが必要なオブジェクトを構築
        can_server_.emplace(can_bus_, board_id);
        motor_.emplace(gate_driver_, encoder_, *can_server_);

        // 制御タイマー (htim6) 割り込み開始
        HAL_TIM_Base_Start_IT(&htim6);
    }

    /** @brief メインループ (全処理は割り込み駆動のため空) */
    void loop() {}

    /**
     * @brief CAN受信割り込みハンドラ
     *        CANBus::update() が受信フレームを各デバイスへルーティングする
     */
    void on_can_rx(FDCAN_HandleTypeDef* /*hfdcan*/)
    {
        can_bus_.update();
    }

    /**
     * @brief タイマー割り込みハンドラ (htim6 のみ処理)
     *        毎制御周期 (1ms) に MotorController と LED を更新する
     */
    void on_timer(TIM_HandleTypeDef* htim)
    {
        if (htim->Instance != TIM6 || !motor_.has_value()) {
            return;
        }

        // リミットスイッチ状態取得 (bit0 = LIM1)
        const uint8_t limit_sw = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin) ? 1U : 0U;

        motor_->update(CONTROL_DT_S, limit_sw);
        update_leds();
    }

private:
    /**
     * @brief DIP スイッチを読み取りボード ID を返す
     *        DIP4=bit0(LSB), DIP3=bit1, DIP2=bit2, DIP1=bit3(MSB)
     * @return uint8_t ボード ID [0-15]
     */
    static uint8_t read_board_id()
    {
        uint8_t id = 0;
        if (HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin)) id |= 0b0001U;
        if (HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin)) id |= 0b0010U;
        if (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin)) id |= 0b0100U;
        if (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin)) id |= 0b1000U;
        return id;
    }

    /**
     * @brief LED 状態を更新する (タイマー割り込みから毎制御周期呼ぶ)
     *
     * | LED  | 色 | 仕様                            |
     * |------|----|---------------------------------|
     * | LED1 | 赤 | 100周期ごとにトグル (制御周期確認) |
     * | LED2 | 赤 | 逆回転時点灯                     |
     * | LED3 | 青 | 回転時点灯                       |
     * | LED4 | 緑 | setup() 以降常時点灯              |
     */
    void update_leds()
    {
        // LED1: 100周期ごとにトグル
        ++led1_count_;
        if (led1_count_ >= LED1_BLINK_CYCLES) {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            led1_count_ = 0;
        }

        // LED4は初期化後点灯
        if (motor_->is_initialized()) {
            HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
        }

        if (!motor_.has_value()) {
            return;
        }

        const float target = motor_->get_target();

        // LED2: 逆回転 (target < 0) のとき点灯
        HAL_GPIO_WritePin(
            LED2_GPIO_Port, LED2_Pin, (target < 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET
        );

        // LED3: 回転中 (target != 0) のとき点灯
        HAL_GPIO_WritePin(
            LED3_GPIO_Port, LED3_Pin, (target != 0.0f) ? GPIO_PIN_SET : GPIO_PIN_RESET
        );
    }

    // --- ハードウェア層 (コンストラクタで安全に生成できる) ---
    gn10_can::drivers::DriverSTM32FDCAN can_driver_;  ///< STM32 CAN ハードウェアドライバ
    gn10_can::CANBus can_bus_;                        ///< CAN バスルーター
    A3921GateDriver gate_driver_;                     ///< A3921 ゲートドライバ
    IncrementalEncoder encoder_;                      ///< インクリメンタルエンコーダ

    // --- 実行時パラメータが必要なオブジェクト (setup() で emplace 構築) ---
    std::optional<gn10_can::devices::MotorDriverServer> can_server_;
    std::optional<gn10_motor::MotorController> motor_;

    // --- LED 状態 ---
    uint32_t led1_count_;  ///< LED1 点滅カウンタ
};

App gn10_app;

}  // namespace

// ---------------------------------------------------------------------------
// C エントリポイント (main.c から呼ばれる)
// ---------------------------------------------------------------------------

void setup()
{
    gn10_app.setup();
}

void loop()
{
    gn10_app.loop();
}

extern "C" {
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs)
{
    gn10_app.on_can_rx(hfdcan);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
    gn10_app.on_timer(htim);
}
}  // extern "C"
