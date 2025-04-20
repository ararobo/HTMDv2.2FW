#include "motor_controller.hpp"
#include <algorithm>

void MotorController::init(uint8_t control_cycle_)
{
    control_cycle = control_cycle_; // 制御周期を設定
    hardware_init();                // ハードウェアの初期化
    reset_pid();                    // PID制御の初期化
    get_count();                    // エンコーダーのカウントを取得
}

int16_t MotorController::trapezoidal_control(int16_t output, uint8_t max_acceleration)
{
    int16_t acceleration = output - prev_out; // 加速度を計算
    if (acceleration < -max_acceleration)
    {
        output = prev_out - max_acceleration;
    }
    else if (acceleration > max_acceleration)
    {
        output = prev_out + max_acceleration;
    }
    prev_out = output;
    return output;
}

float MotorController::calculate_pid(float target, float now_value)
{
    // ターゲットが0の場合は出力を0にする
    if (target == 0.0f)
    {
        i_out = 0.0f;
        prev_error = 0.0f;
        return 0.0f;
    }
    i_out = std::clamp(i_out, -(float)max_output, (float)max_output); // I制御の出力を制限
    float error = target - now_value;                                 // エラーを計算
    float p_out = error;                                              // P制御
    i_out += error * (float)control_cycle;                            // I制御
    float d_out = (error - prev_error) / (float)control_cycle;        // D制御
    prev_error = error;                                               // 前回のエラーを保存
    return p_out * Kp + i_out * Ki + d_out * Kd;                      // PID制御
}

void MotorController::set_pid_gain(float p_gain, float i_gain, float d_gain)
{
    Kp = p_gain;
    Ki = i_gain;
    Kd = d_gain;
    reset_pid(); // PID制御の初期化
}

void MotorController::reset()
{
    reset_pid();                 // PID制御の初期化
    reset_trapezoidal_control(); // 台形制御の初期化
}

void MotorController::reset_pid()
{
    i_out = 0.0f;
    prev_error = 0.0f;
}

void MotorController::reset_trapezoidal_control()
{
    prev_out = 0;
}