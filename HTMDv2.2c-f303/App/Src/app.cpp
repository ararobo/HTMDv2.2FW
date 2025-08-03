/**
 * @file app.cpp
 * @author aiba-gento Watanabe-Koichiro
 * @brief MDのファームウェアのメインクラス
 * @version 1.0
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "app.hpp"
#include "tim.h"
#include "dc_motor_controller.hpp"
#include "serial_printf.hpp"
#define BOARD_TYPE 0x00

CANDriver can;
MotorController motor_controller;

void App::setup()
{
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);

    // MDのIDを更新
    update_md_id();

    // CAN通信の初期化
    can.set_board_id(md_id);
    can.init(0, 0); // フィルタIDとマスクを設定

    // モーターの初期化
    motor_controller.init();

    // エンコーダーサンプリング用のタイマーを開始
    HAL_TIM_Base_Start_IT(&htim6);

    log_printf(LOG_INFO, "App initialized.\n");
}

void App::loop()
{
    if (initialized)
    {
        // 目標値の更新
        if (can.get_target(&target))
        {
            update_target_count = 0;
            log_printf(LOG_DEBUG, "Target: %f\n", target);
        }
        else
        {
            update_target_count++;
        }
        control_motor(); // モーター制御
    }

    update_md_config(); // MDの設定を更新(init)
    update_gain(0);     // Pゲイン
    update_gain(1);     // Iゲイン
    update_gain(2);     // Dゲイン

    // リミットスイッチ等の状態を取得し、CANで送信
    if (limit_switch != HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin))
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin) & 0b1;
        can.send_limit_switch(limit_switch);
    }
    else if (loop_count % 10 == 0) // 10周期ごとにリミットスイッチの状態を送信
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin) & 0b1;
        can.send_limit_switch(limit_switch);
    }

    if (loop_count > loop_count_max)
    {
        if (initialized)
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        loop_count = 0;
    }
    else
    {
        loop_count++;
    }

    wait_for_next_period(); // 制御周期に合わせる
}

void App::timer_callback()
{
    if (md_config.encoder_type == 1 && md_config.encoder_period > 0)
    {
        if (timer_count > md_config.encoder_period)
        {
            now_value = motor_controller.sample_encoder(); // エンコーダーのサンプリング
            can.send_encoder(now_value);                   // エンコーダーの値をCANで送信
            timer_count = 0;
        }
        else
        {
            timer_count++;
        }
    }
}

void App::can_callback_process(CAN_HandleTypeDef *hcan)
{
    can.can_callback_process(hcan);
}

void App::control_motor()
{
    if (update_target_count < update_target_count_max)
    {
        // リミットスイッチによる制御
        if (limit_switch_control())
        {
            motor_controller.stop();
        }
        else
        {
            motor_controller.run(target, now_value); // モーターの制御を行う
        }
    }
    else // 長時間、目標値が更新されない場合、出力を0にする
    {
        motor_controller.stop();
    }

    // LEDの点灯制御
    if (target != 0) // 目標値が0出ない場合はLED3を点灯
    {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    if (target < 0) // 目標値が負の場合はLED2を点灯
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }
}

bool App::limit_switch_control()
{
    if (!limit_switch)
    {
        limit_stop = true;
    }

    switch (md_config.limit_switch_behavior)
    {
    case 0: // 何もしない
        break;

    case 1: // リミットスイッチが押されたら、制御値がゼロになるまでモーターを回さない
        if (limit_switch && target == 0)
        {
            limit_stop = false;
        }
        if (limit_switch && limit_stop)
        {
            return true;
        }
        break;

    case 2: // リミットスイッチが押されたら、正回転のみ停止する
        if (limit_switch && target > 0)
        {
            return true;
        }
        break;

    case 3:
        // リミットスイッチが押されたら、負回転のみ停止する
        if (limit_switch && target < 0)
        {
            return true;
        }
        break;

    case 4: // リミットスイッチ１で正回転を停止し、リミットスイッチ２で逆回転を停止する
        if (limit_switch & 0b1 && target > 0)
        {
            return true;
        }
        if (limit_switch & 0b10 && target < 0)
        {
            return true;
        }
        break;

    default:
        break;
    }
    return false;
}

void App::update_md_config()
{
    // MDの設定を取得
    if (can.get_init(&md_config))
    {
        motor_controller.set_config(md_config); // モータードライバの設定を更新
        motor_controller.reset();               // 台形制御とPID制御とエンコーダーの初期化

        log_printf(LOG_INFO, "md_canfig.max_output:%d\\n", md_config.max_output);
        log_printf(LOG_INFO, "md_config.max_acceleration:%d\n", md_config.max_acceleration);
        log_printf(LOG_INFO, "md_config.control_period:%d\n", md_config.control_period);
        log_printf(LOG_INFO, "md_config.encoder_period:%d\n", md_config.encoder_period);
        log_printf(LOG_INFO, "md_config.encoder_type:%d\n", md_config.encoder_type);
        log_printf(LOG_INFO, "md_config.limit_switch_behavior:%d\n", md_config.limit_switch_behavior);
        log_printf(LOG_INFO, "md_config.option:%d\n", md_config.option);

        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        initialized = true; // 初期化フラグを立てる

        log_printf(LOG_INFO, "MD initialized.\n");
    }
}

void App::update_gain(uint8_t gain_type)
{
    // ゲインの取得
    if (can.get_gain(gain_type, &pid_gain[gain_type]))
    {
        // 確認のためゲインを送り返す
        can.send_gain(gain_type, pid_gain[gain_type]);
        // PIDゲインを更新
        motor_controller.set_pid_gain(pid_gain[0], pid_gain[1], pid_gain[2]);

        log_printf(LOG_INFO, "Gain %d updated: %f\n", gain_type, pid_gain[gain_type]);
    }
}

void App::wait_for_next_period()
{
    // 制御周期に合わせて待機する
    uint32_t current_tick = HAL_GetTick();
    uint32_t elapsed = current_tick - last_tick;

    // オーバーフロー対策とタイムアウト対策
    if (elapsed < md_config.control_period && elapsed < 1000) // 1秒以内の場合のみ待機
    {
        while ((HAL_GetTick() - last_tick) < md_config.control_period)
        {
        }
    }
    last_tick = HAL_GetTick();
}

void App::update_md_id()
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
    md_id = id;
}

App::App()
{
    md_config.control_period = 5;
    md_config.encoder_period = 5;
    md_config.max_output = 0;
    md_config.max_acceleration = 0;
    md_config.encoder_type = 0;
    md_config.limit_switch_behavior = 0;
    md_config.option = 0;
    md_id = 0;
    target = 0.0f;
    now_value = 0.0f;
    limit_switch = 0;
    update_target_count = 0;
    update_target_count_max = 100;
    timer_count = 0;
    loop_count = 0;
    loop_count_max = 100;
    last_tick = 0;
    initialized = false;
    pid_gain[0] = 0.0f;
    pid_gain[1] = 0.0f;
    pid_gain[2] = 0.0f;
    limit_stop = false;
}