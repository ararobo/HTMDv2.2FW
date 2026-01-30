#pragma once

#include "gn10_can/core/can_bus.hpp"
#include "gn10_can/drivers/driver_interface.hpp"
#include "common/logic/motor_manager.hpp"
#include "common/interfaces/gate_driver_interface.hpp"
#include "common/interfaces/encoder_interface.hpp"
#include "common/interfaces/indicator_interface.hpp"

namespace common::core {

/**
 * @brief アプリケーション全体の統合管理クラス
 * 
 * ハードウェア・ドライバの依存性を注入され、
 * メインループ内でMotorManagerやCANBusの更新をオーケストレーションします。
 */
class App {
  public:
    /**
     * @brief コンストラクタ
     * 
     * @param gate_driver ゲートドライバ（PWM）インターフェース
     * @param encoder エンコーダインターフェース
     * @param indicator インジケータ（LED）インターフェース
     * @param can_driver CAN物理層ドライバインターフェース
     * @param device_id このデバイスのCAN ID
     */
    App(interfaces::GateDriverInterface& gate_driver,
        interfaces::EncoderInterface& encoder,
        interfaces::IndicatorInterface& indicator,
        gn10_can::drivers::DriverInterface& can_driver,
        uint8_t device_id);

    /**
     * @brief 初期化処理
     * ハードウェア初期化・通信初期化を行う
     */
    void init();

    /**
     * @brief 定期実行処理
     * メインループから呼び出される
     */
    void update();

  private:
    // CAN通信管理
    gn10_can::CANBus can_bus_;
    
    // モーター制御ロジック
    logic::MotorManager motor_manager_;
};

}  // namespace common::core
