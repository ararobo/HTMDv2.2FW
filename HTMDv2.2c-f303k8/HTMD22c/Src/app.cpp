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

union md_mode_ // MDのモード
{
    struct
    {
        uint8_t device_code;                   // デバイスコード
        unsigned char limit_switch : 1;        // リミットスイッチ
        unsigned char incremental_encoder : 1; // インクリメンタルエンコーダ
        unsigned char absolute_encoder : 1;    // アブソリュートエンコーダ
        unsigned char pid : 1;                 // PID制御
        unsigned char brake : 1;               // ブレーキ
        unsigned char reverse_encoder : 1;     // エンコーダの反転
        unsigned char none2 : 1;               // 予約
        unsigned char none3 : 1;               // 予約
        uint8_t max_acceleration;              // 最大加速度
        uint8_t reporting_cycle;               // 周期
        uint16_t max_output;                   // 最大出力
        uint16_t motor_transfer_cofficient;    // モーターの伝達関数(100倍)
    };
    uint8_t code[6];
} md_mode;

CANDataManager myCAN;
MotorController motor;

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
    serial_printf("default\n");
    serial_printf("Kp: %f\n", Kp);
    serial_printf("Ki: %f\n", Ki);
    serial_printf("Kd: %f\n", Kd);
    serial_printf("report: %d\n", reporting_cycle);
    serial_printf("acces: %d\n", max_acceleration);
    serial_printf("abs_enc: %d\n", enable_absolute_encoder);
    serial_printf("brake: %d\n", enable_brake);
    serial_printf("inc_enc: %d\n", enable_incremental_encoder);
    serial_printf("limSw: %d\n", enable_limit_switch);
    serial_printf("pid: %d\n", enable_pid);
    serial_printf("max_out: %d\n", max_output);
    serial_printf("motor_transfer_cofficient: %f\n", motor_transfer_cofficient);
    // peripheral init
    myCAN.init(md_id);                     // CAN通信の初期化
    motor.init(max_output, control_cycle); // モーターの初期化
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
        reporting_cycle = md_mode.reporting_cycle;
        max_acceleration = md_mode.max_acceleration;
        enable_absolute_encoder = md_mode.absolute_encoder;
        enable_brake = md_mode.brake;
        enable_incremental_encoder = md_mode.incremental_encoder;
        enable_limit_switch = md_mode.limit_switch;
        enable_pid = md_mode.pid;
        max_output = md_mode.max_output;
        motor_transfer_cofficient = float(md_mode.motor_transfer_cofficient) / 100.0f; // 100倍されているので100で割る
        if (max_output > 3199)                                                         // 最大出力が3199を超えていたら
        {
            max_output = 3199;
        }
        // print
        serial_printf("updated md mode\n");
        serial_printf("report: %d\n", reporting_cycle);
        serial_printf("acces: %d\n", max_acceleration);
        serial_printf("abs_enc: %d\n", enable_absolute_encoder);
        serial_printf("brake: %d\n", enable_brake);
        serial_printf("inc_enc: %d\n", enable_incremental_encoder);
        serial_printf("limSw: %d\n", enable_limit_switch);
        serial_printf("pid: %d\n", enable_pid);
        serial_printf("max_out: %d\n", max_output);
        resetControlVal(); // 制御値をリセット
    }
    // 初期化されていればMDの状態を送信する
    if (initialized)
    {
        if (enable_incremental_encoder || enable_limit_switch) // インクリメンタルエンコーダかリミットスイッチが有効なら
        {
            myCAN.sendMDStatus(HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin), false, encoder_value); // エンコーダの値を送信
            serial_printf("encoder: %d\n", encoder_value);                                        // エンコーダの値を表示
        }

        HAL_Delay(reporting_cycle); // 周期的に送信
    }
    else
    {
        if (myCAN.getMDInit()) // 初期化コマンドが送られてきたら
        {
            myCAN.sendReInitCommand(); // レスポンス
            initialized = true;        // 初期化されたら
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