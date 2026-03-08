/**
 * @file motor_controller.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief DCモーター制御クラス
 * @version 0.2.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <array>
#include <cstdint>

#include "gn10_can/devices/motor_driver_server.hpp"
#include "gn10_can/devices/motor_driver_types.hpp"
#include "gn10_motor/acceleration_limiter.hpp"
#include "gn10_motor/i_encoder.hpp"
#include "gn10_motor/i_gate_driver.hpp"
#include "gn10_motor/pid.hpp"

namespace gn10_motor {

/**
 * @brief DCモーター制御クラス
 *
 * IGateDriver / IEncoder / MotorDriverServer をDIとして受け取り、
 * PID制御 + 加速度制限 + リミットスイッチ処理を組み合わせてモーターを制御する。
 *
 * CAN通信は MotorDriverServer を経由し、設定・目標値・ゲインを受け取り
 * エンコーダのフィードバック値を送り返す。
 *
 * CAN受信割り込み  ： CANBus::update() (MotorControllerは不要)
 *
 * タイマー割り込み ： MotorController::update(dt_s, limit_switch_state)
 */
class MotorController
{
public:
    /**
     * @brief コンストラクタ
     * @param driver   ゲートドライバ実装への参照
     * @param encoder  エンコーダ実装への参照
     * @param can_server MotorDriverServer への参照
     */
    MotorController(
        IGateDriver& driver, IEncoder& encoder, gn10_can::devices::MotorDriverServer& can_server
    );

    /**
     * @brief 制御ループを1ステップ実行する
     *
     * @param dt_s              制御周期 [s]
     * @param limit_switch_state リミットスイッチ状態 (ビットマップ、bit0=SW1, bit1=SW2)
     *
     * @details タイマー割り込みから毎制御周期呼ぶこと。内部では以下を順番に処理する。
     * 1. CAN から設定/ゲイン/目標値を polling
     * 2. エンコーダ読み取り
     * 3. PID (または オープンループ) 演算
     * 4. 加速度制限
     * 5. リミットスイッチによる出力制限
     * 6. ゲートドライバへ出力
     * 7. フィードバック値を CAN 送信
     */
    void update(float dt_s, uint8_t limit_switch_state = 0);

    /**
     * @brief モーターを即時停止し、制御器をリセットする
     */
    void stop();

    /**
     * @brief 制御器の内部状態をリセットする (設定・初期化状態は維持)
     */
    void reset();

    /**
     * @brief CAN init パケット受信済みか否かを返す
     * @return true 初期化済み (制御動作中)
     */
    bool is_initialized() const
    {
        return initialized_;
    }

    /**
     * @brief 現在の目標値を返す (LED 制御などの UI 用途向け)
     * @return float 最後に受け取った目標値
     */
    float get_target() const
    {
        return target_;
    }

private:
    // --- DI で注入されるハードウェア依存オブジェクト ---
    IGateDriver& driver_;
    IEncoder& encoder_;
    gn10_can::devices::MotorDriverServer& can_server_;

    // --- 制御アルゴリズム ---
    PID<float> pid_;
    AccelerationLimiter<float> accel_limiter_;

    // --- 状態 ---
    float target_;          ///< CAN から受け取った目標値
    float feedback_value_;  ///< エンコーダから計算したフィードバック値 [rad/s or rad]
    bool initialized_;      ///< init パケット受信後に true になる

    // --- 設定 ---
    gn10_can::devices::MotorConfig config_;
    std::array<float, static_cast<std::size_t>(gn10_can::devices::GainType::Count)> gains_;

    // --- タイムアウト管理 ---
    uint32_t no_target_count_;
    static constexpr uint32_t NO_TARGET_TIMEOUT_CYCLES = 100;  ///< この周期数だけ更新がなければ停止

    // --- 内部処理 ---

    /**
     * @brief CAN を polling し、新しい設定/ゲイン/目標値を適用する
     */
    void poll_can();

    /**
     * @brief 設定変更時に PIDConfig と AccelerationLimiter を再構築する
     */
    void apply_config_to_controllers();

    /**
     * @brief リミットスイッチによる出力制限
     * @param duty           制限前の出力値 [-1.0, 1.0]
     * @param limit_sw_state リミットスイッチ状態 (ビットマップ)
     * @return float         制限後の出力値 [-1.0, 1.0]
     */
    float apply_limit_switch(float duty, uint8_t limit_sw_state) const;

    /**
     * @brief エンコーダタイプに応じてフィードバック値を更新する
     * @param count read_and_reset_count() の戻り値
     * @param dt_s  制御周期 [s]
     * @return float フィードバック値 (速度 [rad/s] または 積算角度 [rad])
     *               Absolute タイプは未対応のため 0.0f を返し出力を停止する
     */
    float compute_feedback(int16_t count, float dt_s);
};

}  // namespace gn10_motor
