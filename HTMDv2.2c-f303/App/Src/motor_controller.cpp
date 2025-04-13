#include "motor_controller.hpp"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

void MotorController::init(uint8_t control_cycle_)
{
    // 変数の初期化
    control_cycle = control_cycle_;
    // モータードライバの初期化
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET); // ブレーキ有効
    // エンコーダーの初期化
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    reset_pid(); // PID制御の初期化
    get_count();
}

void MotorController::run(int16_t output, uint16_t max_output)
{
    // 出力を最大出力に制限
    output = saturate(int(output), -int(max_output), int(max_output));
    // 出力が負の場合は回転方向を反転
    if (output < 0)
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_RESET);
        output = -output;
    }
    else
    {
        HAL_GPIO_WritePin(PHASE_GPIO_Port, PHASE_Pin, GPIO_PIN_SET);
    }
    // 出力を設定
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, output);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 3200);
}

int16_t MotorController::get_count()
{
    // エンコーダのカウントを取得
    uint16_t enc_buff = TIM1->CNT;
    TIM1->CNT = 0;
    return static_cast<int16_t>(enc_buff); // カウントをint16_tに変換
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
    i_out = saturate(i_out, -(float)max_output, (float)max_output); // I制御の出力を制限
    float error = target - now_value;                               // エラーを計算
    float p_out = error;                                            // P制御
    i_out += error * (float)control_cycle;                          // I制御
    float d_out = (error - prev_error) / (float)control_cycle;      // D制御
    prev_error = error;                                             // 前回のエラーを保存
    return p_out * Kp + i_out * Ki + d_out * Kd;                    // PID制御
}

void MotorController::set_pid_gain(float p_gain, float i_gain, float d_gain)
{
    Kp = p_gain;
    Ki = i_gain;
    Kd = d_gain;
    reset_pid(); // PID制御の初期化
}

void MotorController::reset_pid()
{
    i_out = 0.0f;
    prev_error = 0.0f;
}

template <typename T>
T MotorController::saturate(T value, T min_value, T max_value)
{
    if (value < min_value)
    {
        return min_value;
    }
    else if (value > max_value)
    {
        return max_value;
    }
    else
    {
        return value;
    }
}

void MotorController::set_brake(bool brake)
{
    if (brake)
    {
        HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SR_GPIO_Port, SR_Pin, GPIO_PIN_RESET);
    }
}
