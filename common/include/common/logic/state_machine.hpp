#pragma once
#include <cstdint>

namespace common::logic {

enum class SystemState : uint8_t {
    UNINITIALIZED,  // 起動直後
    CONFIG_WAIT,    // 設定待ち（安全のため、設定を受信するまで動かない）
    IDLE,           // 準備完了・PWM OFF
    RUNNING,        // 制御中・PWM ON
    LIMIT_STOP,     // リミットスイッチ検知による停止
    ERROR,          // 異常発生
};

enum class ControlMode : uint8_t {
    DUTY_CYCLE,
    SPEED_PID,
    POSITION_PID,
};

class StateMachine {
 public:
    StateMachine()
        : current_state_(SystemState::UNINITIALIZED),
          control_mode_(ControlMode::DUTY_CYCLE) {}

    SystemState get_state() const { return current_state_; }

    void init_complete() {
        if (current_state_ == SystemState::UNINITIALIZED) {
            current_state_ = SystemState::CONFIG_WAIT;
        }
    }

    // 設定完了（CANからの初期設定受信）
    void complete_config() {
        if (current_state_ == SystemState::CONFIG_WAIT) {
            current_state_ = SystemState::IDLE;
        }
    }

    // 制御開始
    bool start_control() {
        if (current_state_ == SystemState::IDLE) {
            current_state_ = SystemState::RUNNING;
            return true;
        }
        return false; // エラー中や設定待ち中は開始できない
    }

    // 通常停止 & リミット停止からの復帰
    void stop_control() {
        // RUNNING中だけでなく、LIMIT_STOPからもIDLEに戻れるようにする
        if (current_state_ == SystemState::RUNNING || 
            current_state_ == SystemState::LIMIT_STOP) {
            current_state_ = SystemState::IDLE;
        }
    }

    // リミットスイッチ検知
    void trigger_limit_switch() {
        // IDLE中でもリミット検知状態に遷移させるべき（安全のため）
        if (current_state_ == SystemState::RUNNING || 
            current_state_ == SystemState::IDLE ||
            current_state_ == SystemState::CONFIG_WAIT) {
            current_state_ = SystemState::LIMIT_STOP;
        }
    }

    void trigger_error() { current_state_ = SystemState::ERROR; }

    void clear_error() {
        if (current_state_ == SystemState::ERROR) {
            // エラー復帰時はまずIDLEへ（再設定を要求する場合はCONFIG_WAITへ戻す設計もあり）
            current_state_ = SystemState::IDLE;
        }
    }

    bool set_control_mode(ControlMode mode) {
        // 安全のため、IDLE（停止中）以外でのモード変更を禁止する
        // CONFIG_WAIT中も変更可能とする
        if (current_state_ != SystemState::IDLE && 
            current_state_ != SystemState::CONFIG_WAIT &&
            current_state_ != SystemState::UNINITIALIZED) {
            return false; 
        }
        control_mode_ = mode;
        return true;
    }
    
    ControlMode get_control_mode() const { return control_mode_; }
    
    // ヘルパー関数: PWMを出力して良い状態か？
    bool is_active() const {
        return current_state_ == SystemState::RUNNING;
    }

 private:
    SystemState current_state_;
    ControlMode control_mode_;
};

}  // namespace common::logic