#include "app.hpp"
#include "tim.h"
#include "motor_controller.hpp"
#include "serial_printf.hpp"
#define FW_VERSION 0x00
#define BOARD_TYPE 0x00

CANDriver can_driver(0, BOARD_TYPE, FW_VERSION);
MotorController motor_controller;

void App::init()
{
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    // MDのIDを更新
    update_md_id();

    // CAN通信の初期化
    can_driver.set_board_id(md_id);
    can_driver.init();
    // エンコーダー用タイマー開始
    HAL_TIM_Base_Start_IT(&htim6);

    log_printf(LOG_INFO, "App initialized.\n");
}

void App::main_loop()
{
    if (initialized)
    {
        // 目標値の更新
        if (can_driver.get_target(&target))
        {
            update_target_count = 0;
            log_printf(LOG_DEBUG, "Target: %d\n", target);
        }
        else
        {
            update_target_count++;
        }
        control_motor(); // モーター制御
    }

    update_md_config(); // MDの設定を更新(init)

    update_gain(0); // Pゲイン
    update_gain(1); // Iゲイン
    update_gain(2); // Dゲイン

    // リミットスイッチ等の状態を取得し、CANで送信
    if (limit_switch != HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin))
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin) & 0b1;
        can_driver.send_limit_switch(limit_switch);
    }
    else if (loop_count % 10 == 0) // 10msごとにリミットスイッチの状態を送信
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin) & 0b1;
        can_driver.send_limit_switch(limit_switch);
    }

    if (loop_count > loop_count_max)
    {
        if (initialized)
        {
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
        }
        else
        {
            // 初期化されていない場合、initパケット（MDの情報）を送信
            can_driver.send_init(BOARD_TYPE);
        }
        loop_count = 0;
    }
    else
    {
        loop_count++;
    }

    wait_for_next_period(); // 制御周期に合わせる
}

void App::timer_task()
{
    // encoder_periodの周期でエンコーダをサンプリングし、CANで送信
    if (timer_count > md_config.encoder_period)
    {
        encoder = motor_controller.get_count();
        can_driver.send_encoder(encoder);
        timer_count = 0;
    }
    else
    {
        timer_count++;
    }
}

void App::can_callback_process(CAN_HandleTypeDef *hcan)
{
    can_driver.can_callback_process(hcan);
}

void App::control_motor()
{
    if (update_target_count < update_target_count_max)
    {
        // 出力値の更新
        if (pid_gain[0] != 0.0f && target != 0)
        {
            // PID制御を行う
            output = motor_controller.calculate_pid(target, encoder) * (float)md_config.max_output;
        }
        else
        {
            // 目標値をそのまま出力値とする
            output = target;
        }

        output = motor_controller.trapezoidal_control(output, md_config.max_acceleration);
    }
    else // 長時間、目標値が更新されない場合、出力を0にする
    {
        output = 0;
        motor_controller.reset();
    }

    // LEDの点灯制御
    if (output != 0)
    {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    }
    if (output < 0)
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    }

    // リミットスイッチによる制御
    if (limit_switch_control())
    {
        output = 0;
        motor_controller.reset();
    }
    else
    {
        // モーターを回す
        motor_controller.run(output, md_config.max_output);
    }

    // モーターを回す
    motor_controller.run(output, md_config.max_output);
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
    if (can_driver.get_init(&md_config))
    {
        log_printf(LOG_INFO, "md_canfig.max_output:%d\\n", md_config.max_output);
        log_printf(LOG_INFO, "md_config.max_acceleration:%d\n", md_config.max_acceleration);
        log_printf(LOG_INFO, "md_config.control_period:%d\n", md_config.control_period);
        log_printf(LOG_INFO, "md_config.encoder_period:%d\n", md_config.encoder_period);
        log_printf(LOG_INFO, "md_config.encoder_type:%d\n", md_config.encoder_type);
        log_printf(LOG_INFO, "md_config.limit_switch_behavior:%d\n", md_config.limit_switch_behavior);
        log_printf(LOG_INFO, "md_config.option:%d\n", md_config.option);

        // モーター制御の初期化
        motor_controller.init(md_config.control_period);

        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        initialized = true;
        log_printf(LOG_INFO, "MD initialized.\n");
    }
}

void App::update_gain(uint8_t gain_type)
{
    // ゲインの取得
    if (can_driver.get_gain(gain_type, &pid_gain[gain_type]))
    {
        // 確認のためゲインを送り返す
        can_driver.send_gain(gain_type, pid_gain[gain_type]);
        // PIDゲインを更新
        motor_controller.set_pid_gain(pid_gain[0], pid_gain[1], pid_gain[2]);

        log_printf(LOG_INFO, "Gain %d updated: %f\n", gain_type, pid_gain[gain_type]);
    }
}

void App::wait_for_next_period()
{
    // 制御周期に合わせて待機する
    while ((HAL_GetTick() - last_tick) < md_config.control_period)
    {
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
    target = 0;
    output = 0;
    encoder = 0;
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