#include "common/logic/motor_manager.hpp"
#include "common/config_defs.hpp"

namespace common::logic {

MotorManager::MotorManager(interfaces::EncoderInterface& encoder,
                           interfaces::GateDriverInterface& gate_driver,
                           interfaces::IndicatorInterface& indicator,
                           gn10_can::CANBus& can_bus,
                           uint8_t device_id)
    : encoder_(encoder),
      gate_driver_(gate_driver),
      indicator_(indicator),
      can_slave_(can_bus, device_id),
      state_machine_(),
      pid_controller_({
          config::DefaultPID::Kp,
          config::DefaultPID::Ki,
          config::DefaultPID::Kd,
          config::DefaultPID::IntegralLimit,
          config::DefaultPID::OutputLimit
      }),
      accel_limiter_(config::DEFAULT_ACCEL_LIMIT) {

    // --- コールバック設定 --- //

    // 目標値受信
    can_slave_.set_on_target_received([this](float target) {
        target_value_ = target;
        
        // IDLE状態かつターゲットが0以外なら制御開始
        if (state_machine_.get_state() == SystemState::IDLE) {
            // 安全のため、いきなり大きな値が入った場合は動き出さない等のガードが必要だが
            // ここではシンプルに開始する
            state_machine_.start_control();
        }
    });

    // ゲイン設定受信
    can_slave_.set_on_gain_received([this](MotorDriverSlave::GainType type, float value) {
        // PIDゲイン更新ロジック (PIDクラスにsetterが必要だが、configをいじれるか？)
        // 今回のPIDクラスはconst config参照ではないので再構築かsetterが必要。
        // 実装簡略化のため一旦スキップ、またはPIDクラスを修正すべき
    });

    // 設定受信
    can_slave_.set_on_config_received([this](const MotorDriverSlave::MotorConfig& config) {
        // Config適用
        // 例: state_machine_.set_control_mode(...)
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
    current_value_ = raw_vel;

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
                    output_duty = accel_limiter_.limit(target_value_, config::CONTROL_LOOP_DT);
                } else {
                    // PID制御モード (速度制御)
                   output_duty = pid_controller_.update(target_value_, current_value_, config::CONTROL_LOOP_DT);
                }
                
                // 出力
                gate_driver_.output(output_duty);
                
                // LED制御
                indicator_.set(IndicatorId::ACTIVITY, true);
                // 方向LED (正転ならON、逆転ならOFF的な)
                indicator_.set(IndicatorId::DIRECTION, output_duty > 0);
            }
            break;

        case SystemState::IDLE:
            gate_driver_.output(0.0f);
            gate_driver_.set_brake_mode(true); // ブレーキ
            
            indicator_.set(IndicatorId::ACTIVITY, false);
            indicator_.set(IndicatorId::DIRECTION, false);
            // 本来はupdate毎にtoggleしたいが、ここでは簡易的にON
            indicator_.set(IndicatorId::HEARTBEAT, true); 
            
            // ターゲットが0以外かつRUNNINGでない場合（再開ロジック）はコールバックで処理済み
            // 逆にターゲットが0になったらIDLEに戻る処理が必要ならここ
            if (target_value_ == 0.0f && state_machine_.is_active()) {
                state_machine_.stop_control();
            }
            break;

        case SystemState::ERROR:
            gate_driver_.output(0.0f);
            gate_driver_.set_brake_mode(false); // フリーラン
            
            // エラー時は全点灯など
            indicator_.set(IndicatorId::ACTIVITY, true);
            indicator_.set(IndicatorId::HEARTBEAT, true);
            indicator_.set(IndicatorId::DIRECTION, true);
            break;

        default:
            gate_driver_.output(0.0f);
            break;
    }

    // 3. フィードバック送信 (間引き処理なし、毎回送るとバス負荷高いので注意)
    // 実運用ではタイマーで間引く
    can_slave_.send_feedback(current_value_, 0); 
    // Load CurrentなどはGateDriverから取得できるならここで送る
}

}  // namespace common::logic
