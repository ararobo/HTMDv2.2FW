#include "dc_motor_controller.hpp"
#include "a3921.hpp"
#include "incremental_encoder.hpp"
#include "trapezoidal_controller.hpp"
#include "pid_calculator.hpp"
#include <algorithm>

GateDriver gate_driver(3199);              // 最大デューティ比を3200に設定
IncremantalEncoder encoder(65535);         // エンコーダの初期化
TrapezoidalController trapezoidal_control; // 台形制御の初期化
PIDCalculator pid_calculator(0.001f);      // PID制御の初期化

MotorController::MotorController()
{
    // コンストラクタの初期化
    reset();
}

void MotorController::init()
{
    // ハードウェアの初期化
    gate_driver.hardware_init();
    encoder.hardware_init();
    gate_driver.set_brake(true); // ブレーキをかける

    reset();
}

void MotorController::reset()
{
    // モーターのリセット
    trapezoidal_control.reset_trapezoidal_control();
    pid_calculator.reset_pid();
}

void MotorController::run(int16_t output)
{
    // PID制御ゲインを取得
    float p_gain, _gain;
    pid_calculator.get_pid_gain(&p_gain, &_gain, &_gain);

    // Pゲインが0でない場合、PID制御を行う
    if (p_gain != 0.0f && output != 0 && md_config.encoder_type != 0)
    {
        // PID制御を行う
        output = static_cast<int16_t>(pid_calculator.calculate_pid(output, encoder_count) * md_config.max_output);
    }

    // 台形制御を行う
    output = trapezoidal_control.trapezoidal_control(output, md_config.max_acceleration);
    // 出力値を制限する
    output = std::clamp(output, int16_t(-md_config.max_output), int16_t(md_config.max_output));
    // モーターの出力を設定する
    gate_driver.output(output);
}

void MotorController::set_config(md_config_t config)
{
    // モータードライバの設定を更新する
    md_config = config;
    pid_calculator.set_dt(config.encoder_period / 1000.0f); // 制御周期を設定
}

void MotorController::stop()
{
    // モーターを停止する
    gate_driver.output(0);
    gate_driver.set_brake(true); // ブレーキをかける
    reset();
}

void MotorController::set_pid_gain(float p_gain, float i_gain, float d_gain)
{
    // PIDゲインを設定する
    pid_calculator.set_pid_gain(p_gain, i_gain, d_gain);
}

void MotorController::sample_encoder()
{
    // エンコーダーのカウントを取得
    encoder_count = encoder.get_count();
}