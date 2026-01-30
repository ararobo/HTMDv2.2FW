#include "common/logic/motor_manager.hpp"

namespace common::logic {

MotorManager::MotorManager(interfaces::EncoderInterface& encoder,
                           interfaces::GateDriverInterface& gate_driver,
                           interfaces::IndicatorInterface& indicator,
                           gn10_can::CANBus& can_bus,
                           const MotorManagerConfig& config,
                           uint8_t device_id)
    : encoder_(encoder),
      gate_driver_(gate_driver),
      indicator_(indicator),
      can_slave_(can_bus, device_id),
      state_machine_(),
      pid_controller_(config.pid_config),
      accel_limiter_(config.default_accel_limit),
      config_(config) {

    // --- コールバック設定 --- //

    // 目標値受信
    can_slave_.set_on_target_received([this](float target) {
        target_setpoint_ = target;
        
        // IDLE状態かつターゲットが0以外なら制御開始
        if (state_machine_.get_state() == SystemState::IDLE) {
            // 安全のため、いきなり大きな値が入った場合は動き出さない等のガードが必要だが
            // ここではシンプルに開始する
            state_machine_.start_control();
        }
    });

    // ゲイン設定受信
    can_slave_.set_on_gain_received([this](MotorDriverSlave::GainType type, float value) {
        // PIDゲイン更新ロジック
        // config_はconst参照ではなくコピーを持つ設計にすれば変更可能だが、
        // 現状はMotorManagerConfigで初期化している。
        // PIDクラス自体は再構築が可能。
        auto current = config_.pid_config;
        if (type == MotorDriverSlave::GainType::Kp) current.kp = value;
        if (type == MotorDriverSlave::GainType::Ki) current.ki = value;
        if (type == MotorDriverSlave::GainType::Kd) current.kd = value;
        // ...
        
        // PID再設定
        config_.pid_config = current;
        // 積分項をリセットせずにゲインのみ更新する
        pid_controller_.update_config(current);
    });

    // 設定受信
    can_slave_.set_on_config_received([this](const MotorDriverSlave::MotorConfig& config) {
        // Config適用
        // 例: state_machine_.set_control_mode(...)
        
        // 設定完了通知: CONFIG_WAIT -> IDLE へ遷移
        state_machine_.complete_config();
    });
}

void MotorManager::init() {
    bool dri_ok = gate_driver_.init();
    encoder_.init();
    indicator_.init();

    if (dri_ok) {
        state_machine_.init_complete();
    } else {
        state_machine_.trigger_error();
    }
}

void MotorManager::update() {
    // 1. センサー取得
    // エンコーダ値取得 (ControlModeによって位置か速度か変わる)
    float raw_vel = encoder_.get_velocity();
    // EncoderInterface::get_velocity は [rad/s] を返す想定
    measured_value_ = raw_vel;

    // 2. ステートマシン処理
    SystemState current_state = state_machine_.get_state();

    float output_duty = 0.0f;
    using interfaces::IndicatorId;

    switch (current_state) {
        case SystemState::RUNNING:
            {
                // モードに応じた制御計算
                ControlMode mode = state_machine_.get_control_mode();
                if (mode == ControlMode::DUTY_CYCLE) {
                    // Duty制御モード
                    // 急加速防止のためAccelerationLimiterを通す
                    output_duty = accel_limiter_.update(target_setpoint_, config_.control_loop_dt);
                } else {
                    // PID制御モード (速度制御)
                   output_duty = pid_controller_.update(target_setpoint_, measured_value_, config_.control_loop_dt);
                }
                
                // 出力
                gate_driver_.set_duty_cycle(output_duty);
                
                // LED制御
                indicator_.set(IndicatorId::ACTIVITY, true);
                // 方向LED (正転ならON、逆転ならOFF的な)
                indicator_.set(IndicatorId::DIRECTION, output_duty > 0);
            }
            break;

        case SystemState::CONFIG_WAIT:
            gate_driver_.set_duty_cycle(0.0f);
            gate_driver_.set_brake_mode(false); // 設定待ちはフリーラン
            
            indicator_.set(IndicatorId::ACTIVITY, false);
            indicator_.set(IndicatorId::DIRECTION, false);
            indicator_.set(IndicatorId::HEARTBEAT, true); // 点滅させたいが簡易的にON
            break;

        case SystemState::IDLE:
            gate_driver_.set_duty_cycle(0.0f);
            gate_driver_.set_brake_mode(true); // ブレーキ
            
            indicator_.set(IndicatorId::ACTIVITY, false);
            indicator_.set(IndicatorId::DIRECTION, false);
            // 本来はupdate毎にtoggleしたいが、ここでは簡易的にON
            indicator_.set(IndicatorId::HEARTBEAT, true); 
            
            // ターゲットが0以外かつRUNNINGでない場合（再開ロジック）はコールバックで処理済み
            // 逆にターゲットが0になったらIDLEに戻る処理が必要ならここ
            if (target_setpoint_ == 0.0f && state_machine_.is_active()) {
                state_machine_.stop_control();
            }
            break;

        case SystemState::LIMIT_STOP:
            // リミット検知時は安全にブレーキ保持
            gate_driver_.set_duty_cycle(0.0f);
            gate_driver_.set_brake_mode(true);
            
            // 警告表示
            indicator_.set(IndicatorId::ACTIVITY, true);
            indicator_.set(IndicatorId::DIRECTION, true); // 点灯により警告
            break;

        case SystemState::ERROR:
            gate_driver_.set_duty_cycle(0.0f);
            gate_driver_.set_brake_mode(false); // フリーラン
            
            // エラー時は全点灯など
            indicator_.set(IndicatorId::ACTIVITY, true);
            indicator_.set(IndicatorId::HEARTBEAT, true);
            indicator_.set(IndicatorId::DIRECTION, true);
            break;

        default:
            gate_driver_.set_duty_cycle(0.0f);
            break;
    }

    // 3. フィードバック送信 (間引き処理なし、毎回送るとバス負荷高いので注意)
    // 実運用ではタイマーで間引く
    can_slave_.send_feedback(measured_value_, 0); 
    // Load CurrentなどはGateDriverから取得できるならここで送る
}

}  // namespace common::logic
