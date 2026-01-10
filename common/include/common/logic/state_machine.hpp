#pragma once
#include <cstdint>

namespace common::logic {

// シンプルなシステムの状態
enum class SystemState : uint8_t {
    UNINITIALIZED,  // 起動直後
    IDLE,           // 準備完了・PWM OFF・指令待ち
    RUNNING,        // 制御中・PWM ON
    LIMIT_STOP,     // リミットスイッチ検知による停止中（復帰待ち）
    ERROR,          // 異常発生・PWM遮断
};

// 制御モード（Running中の詳細動作）
enum class ControlMode : uint8_t {
    DUTY_CYCLE,    // シンプルなDuty制御
    SPEED_PID,     // 速度制御
    POSITION_PID,  // 位置制御
};

class StateMachine {
  public:
    StateMachine()
        : current_state_(SystemState::UNINITIALIZED), control_mode_(ControlMode::DUTY_CYCLE) {}

    // 状態の取得・設定
    SystemState get_state() const { return current_state_; }

    // イベントハンドラ
    void init_complete() {
        if (current_state_ == SystemState::UNINITIALIZED) {
            current_state_ = SystemState::IDLE;
        }
    }

    void start_control() {
        if (current_state_ == SystemState::IDLE) {
            current_state_ = SystemState::RUNNING;
        }
    }

    void stop_control() {
        if (current_state_ == SystemState::RUNNING) {
            current_state_ = SystemState::IDLE;
        }
    }

    void trigger_limit_switch() {
        if (current_state_ == SystemState::RUNNING) {
            current_state_ = SystemState::LIMIT_STOP;
        }
    }

    void trigger_error() { current_state_ = SystemState::ERROR; }

    void clear_error() {
        if (current_state_ == SystemState::ERROR) {
            current_state_ = SystemState::IDLE;
        }
    }

    // 制御モード設定
    void set_control_mode(ControlMode mode) { control_mode_ = mode; }
    ControlMode get_control_mode() const { return control_mode_; }

  private:
    SystemState current_state_;
    ControlMode control_mode_;
};

}  // namespace common::logic