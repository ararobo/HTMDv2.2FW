#include <cstdio>
#include <cstring>
#include "app.hpp"
#include "motor_controller.hpp"
#include "indicator.hpp"
#include "main.h"
#include "can.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "htmd_mode.hpp"

CANDataManager myCAN;
MotorController motor;
md_mode_t md_mode;

void App::getMDIdFromDispSW(uint8_t *md_id_)
{
    uint8_t id = 0;
    if (HAL_GPIO_ReadPin(DIP4_GPIO_Port, DIP4_Pin))
        id = 1;
    if (HAL_GPIO_ReadPin(DIP3_GPIO_Port, DIP3_Pin))
        id |= 0b10;
    if (HAL_GPIO_ReadPin(DIP2_GPIO_Port, DIP2_Pin))
        id |= 0b100;
    if (HAL_GPIO_ReadPin(DIP1_GPIO_Port, DIP1_Pin))
        id |= 0b1000;
    *md_id_ = id; // ディップスイッチの値をMDのIDに設定
}

void App::init()
{
    getMDIdFromDispSW(&md_id); // ディップスイッチの値をMDのIDに設定
    serial_printf("md_id: %d\n", md_id);
    // peripheral init
    myCAN.init(md_id);                                    // CAN通信の初期化
    motor.init(md_mode.values.max_output, control_cycle); // モーターの初期化
    indicateStanby(true);
    // main timer start
    HAL_TIM_Base_Start_IT(&htim6);
}

void App::mainLoop()
{
    if (myCAN.getPIDGain(&Kp, &Ki, &Kd)) // PIDゲインが更新されていたら
    {
        motor.setPIDGain(Kp, Ki, Kd);    // PIDゲインを更新
        myCAN.sendReInitPID(Kp, Ki, Kd); // レスポンス
        // print
        serial_printf("updated md pid gains\n");
        serial_printf("Kp: %f\n", Kp);
        serial_printf("Ki: %f\n", Ki);
        serial_printf("Kd: %f\n", Kd);
        resetControlVal(); // 制御値をリセット
    }
    if (myCAN.getMDMode(md_mode.code)) // モードが更新されていたら
    {
        myCAN.sendReInitMode(md_mode.code); // レスポンス
        // モードを更新
        motor_transfer_cofficient = float(md_mode.values.motor_transfer_coefficient) / 100.0f; // 100倍されているので100で割る
        if (md_mode.values.max_output > 3199)                                                  // 最大出力が3199を超えていたら
        {
            md_mode.values.max_output = 3199;
        }
        // print
        serial_printf("updated md mode\n");
        serial_printf("incremental_encoder: %d\n", md_mode.flags.incremental_encoder);
        serial_printf("absolute_encoder: %d\n", md_mode.flags.absolute_encoder);
        serial_printf("reverse_encoder: %d\n", md_mode.flags.reverse_encoder);
        serial_printf("limit_switch: %d\n", md_mode.flags.limit_switch);
        serial_printf("brake: %d\n", md_mode.flags.brake);
        serial_printf("pid: %d\n", md_mode.flags.pid);
        serial_printf("torque_control: %d\n", md_mode.flags.torque_control);
        serial_printf("state_temp: %d\n", md_mode.flags.state_temp);
        serial_printf("max_acceleration: %d\n", md_mode.values.max_acceleration);
        serial_printf("max_current: %d\n", md_mode.values.max_current);
        serial_printf("report_rate: %d\n", md_mode.values.report_rate);
        serial_printf("max_output: %d\n", md_mode.values.max_output);
        serial_printf("motor_transfer_coefficient: %f\n", motor_transfer_cofficient);
        resetControlVal(); // 制御値をリセット
    }
    // 初期化されていればMDの状態を送信する
    if (initialized)
    {
        if (md_mode.flags.incremental_encoder || enable_limit_switch) // インクリメンタルエンコーダかリミットスイッチが有効なら
        {
            myCAN.sendMDStatus(HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin), false, encoder_value); // エンコーダの値を送信
            serial_printf("encoder: %d\n", encoder_value);                                        // エンコーダの値を表示
        }

        HAL_Delay(report_rate); // 周期的に送信
    }
    else
    {
        if (myCAN.getMDInit()) // 初期化コマンドが送られてきたら
        {
            initialized = true; // 初期化されたら
            indicateReady(true);
            indicateStanby(false);
            serial_printf("initialized\n");
            resetControlVal(); // 制御値をリセット
        }
        // serial_printf("waiting for init\n");
    }
}

void App::timerTask()
{
    // serial_printf("timer\n");
    // 目標値（出力）を更新する
    if (myCAN.getMotorTarget(&target))
    {
        no_update_count = 0; // 更新されたらカウントをリセット
        serial_printf("target: %d\n", target);
    }
    else
    {
        no_update_count++; // 更新されていない回数をカウント
    }
    // エンコーダーを読む
    if (enable_pid || enable_incremental_encoder) // PID制御が有効 or インクリメンタルエンコーダが有効なら
    {
        if (md_mode.reverse_encoder) // reverse_encoderが有効なら
        {
            encoder_value = -motor.getCount(); // エンコーダのカウントを取得 (反転)
        }
        else
        {
            encoder_value = motor.getCount(); // エンコーダのカウントを取得
        }
    }
    // PID制御をする
    if (enable_pid)
    {
        output += motor.calculatePID(target, encoder_value);          // PID制御
        if ((target < 0 && output > 0) || (target > 0 && output < 0)) // 目標値の符号と出力の符号が異なる場合
        {
            output = 0;
        }
        // 出力を最大出力に制限
        if (abs(output) > abs(target) * motor_transfer_cofficient)
        {
            output = target * motor_transfer_cofficient;
            serial_printf("Warn: output adjusted\n");
        }
    }
    else
    {
        output = target;
    }
    // 安全装置
    output = motor.trapezoidalControl(output, max_acceleration);             // 台形制御
    output = motor.saturate(int(output), -int(max_output), int(max_output)); // 出力を最大出力に制限

    if (no_update_count > no_update_max || !initialized) // 値が長い間更新されていなければ or 初期化されていなければ
    {
        resetControlVal(); // 制御値をリセット
    }
    else
    {
        if (target != 0) // 目標値が0でなければ
        {
            indicateBusy(true);
        }
        else
        {
            resetControlVal(); // 制御値をリセット
        }
        motor.run(output, max_output); // モーターを制御
    }
}
void App::CANCallbackProcess(CAN_HandleTypeDef *hcan_)
{
    myCAN.onReceiveTask(hcan_); // CAN通信のコールバック処理
}

template <typename... Args>
void App::serial_printf(const std::string &fmt, Args... args)
{
    // フォーマットされた文字列の長さを取得
    size_t len = std::snprintf(nullptr, 0, fmt.c_str(), args...);
    // バッファを作成してフォーマットされた文字列を格納
    std::vector<char> buf(len + 1);
    std::snprintf(&buf[0], len + 1, fmt.c_str(), args...);
    // ヌル終端された文字列をUARTに送信
    HAL_UART_Transmit(&huart1, (uint8_t *)&buf[0], len, 0xFF);
}

void App::resetControlVal()
{
    target = 0;
    output = 0;
    motor.resetPID();
    motor.run(0, max_output);
    indicateBusy(false);
}