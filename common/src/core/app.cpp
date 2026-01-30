#include "common/core/app.hpp"

namespace common::core {

App::App(interfaces::GateDriverInterface& gate_driver,
         interfaces::EncoderInterface& encoder,
         interfaces::IndicatorInterface& indicator,
         gn10_can::drivers::DriverInterface& can_driver,
         uint8_t device_id)
    : can_bus_(can_driver),
      motor_manager_(encoder, gate_driver, indicator, can_bus_, device_id) {}

void App::init() {
    // コンポーネントの初期化
    motor_manager_.init();
}

void App::update() {
    // CANのメッセージ配送
    can_bus_.update();
    
    // モーター制御ロジックの更新
    motor_manager_.update();
}

}  // namespace common::core
