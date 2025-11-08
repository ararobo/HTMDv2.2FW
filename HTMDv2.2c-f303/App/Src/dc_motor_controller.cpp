/**
 * @file dc_motor_controller.cpp
 * @author aiba-gento Watanabe-Koichiro
 * @brief DCモーターの制御クラス
 * @version 2.0
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "dc_motor_controller.hpp"
#include "a3921.hpp"
#include "incremental_encoder.hpp"
#include "trapezoidal_controller.hpp"
#include "pid_calculator.hpp"
#include "serial_printf.hpp"
#include <algorithm>

GateDriver gate_driver(3199);                       // 最大デューティ比を3200に設定
IncremantalEncoder encoder;                         // エンコーダの初期化
TrapezoidalController<int16_t> trapezoidal_control; // 台形制御の初期化
PIDCalculator pid_calculator;                       // PID制御の初期化

MotorController::MotorController()
{
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
    trapezoidal_control.reset();
    pid_calculator.reset();
    encoder.reset();       // エンコーダのリセット
    gate_driver.output(0); // 出力を0にする
}

void MotorController::run(float output, float now_value, uint8_t limit_sw_status)
{
    // PID制御ゲインを取得
    float p_gain, _gain;
    pid_calculator.get_pid_gain(&p_gain, &_gain, &_gain);

    // Pゲインが0でない場合、PID制御を行う
    if (p_gain != 0.0f && output != 0 && md_config.encoder_type != 0)
    {
        // PID制御を行う
        output = float(pid_calculator.calculate_pid(output, now_value));
    }
    output = std::clamp(output, -1.0f, 1.0f);
    int16_t duty = output * float(md_config.max_output);

    // 台形制御を行う
    duty = trapezoidal_control.trapezoidal_control(duty, md_config.max_acceleration);
    duty = limit_switch_control(duty, limit_sw_status);
    // 出力値を制限する
    duty = std::clamp(duty, int16_t(-md_config.max_output), (int16_t)md_config.max_output);
    // モーターの出力を設定する
    gate_driver.output(duty);
}

void MotorController::set_config(md_config_t config)
{
    // モータードライバの設定を更新する
    md_config = config;
    pid_calculator.set_dt(config.control_period / 1000.0f);       // 制御周期を設定
    trapezoidal_control.set_control_cycle(config.control_period); // 台形制御の制御周期を設定
}

void MotorController::stop()
{
    // モーターの出力を停止する
    gate_driver.output(0);
    trapezoidal_control.reset(); // 台形制御のリセット
    pid_calculator.reset();      // PID制御のリセット
}

void MotorController::set_pid_gain(float p_gain, float i_gain, float d_gain)
{
    // PIDゲインを設定する
    pid_calculator.set_pid_gain(p_gain, i_gain, d_gain);
}

float MotorController::sample_encoder()
{
    if (md_config.encoder_type == 1)
    {
        return encoder.count_to_angular_velocity(encoder.get_count(), md_config.encoder_period / 1000.0f);
    }
    if (md_config.encoder_type == 3)
    {
        return encoder.total_encoder(encoder.get_count());
    }
    return 0; // エンコーダーがない場合は0を返す
}

int16_t MotorController::limit_switch_control(int16_t duty, uint8_t limit_sw_status)
{
    if (!limit_sw_status)
    {
        limit_stop = true;
    }

    switch (md_config.limit_switch_behavior)
    {
    case 0: // 何もしない
        break;

    case 1: // リミットスイッチが押されたら、制御値がゼロになるまでモーターを回さない
        if (limit_sw_status && duty == 0)
        {
            limit_stop = false;
        }
        if (limit_sw_status && limit_stop)
        {
            return 0;
        }
        break;

    case 2: // リミットスイッチが押されたら、正回転のみ停止する
        if (limit_sw_status && duty > 0)
        {
            return 0;
        }
        break;

    case 3:
        // リミットスイッチが押されたら、負回転のみ停止する
        if (limit_sw_status && duty < 0)
        {
            return 0;
        }
        break;

    case 4: // リミットスイッチ１で正回転を停止し、リミットスイッチ２で逆回転を停止する
        if (limit_sw_status & 0b1 && duty > 0)
        {
            return 0;
        }
        if (limit_sw_status & 0b10 && duty < 0)
        {
            return 0;
        }
        break;

    default:
        break;
    }
    return duty;
}