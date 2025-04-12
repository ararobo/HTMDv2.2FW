#include "app.hpp"
#include "tim.h"
#include "motor_controller.hpp"
#include "serial_printf.hpp"
#define FW_VERSION 0x01
#define BOARD_KIND 0x01

CANDriver can_driver(0, BOARD_KIND, FW_VERSION);
MotorController motor_controller;

App::App()
{
    md_config.control_period = 5;        // ms
    md_config.encoder_period = 5;        // ms
    md_config.max_output = 0;            // 最大出力
    md_config.max_acceleration = 0;      // 台形制御の最大加速 duty/10ms
    md_config.encoder_type = 0;          // 0:無し、1:インクリメンタル、2:アブソリュート
    md_config.limit_switch_behavior = 0; // リミットスイッチの動作設定
    md_config.option = 0;                // 基板の固有機能や使用用途に合わせて決定（未定義）
    md_id = 0;                           // 基板のID
    target = 0;                          // 目標位置
    output = 0;                          // 出力
    encoder = 0;                         // エンコーダのカウント
    limit_switch = 0;                    // リミットスイッチの状態
    update_target_count = 0;             // 目標位置の更新カウント
    update_target_count_max = 100;       // 目標位置の更新カウントの最大値
    timer_count = 0;                     // タイマーカウント
    loop_count = 0;                      // ループカウント
    loop_count_max = 100;                // ループカウントの最大値
    last_tick = 0;                       // 最後のティック
    initialized = false;                 // 初期化フラグ
    pid_gains[0] = 0.0f;                 // PIDゲインの初期値
    pid_gains[1] = 0.0f;                 // PIDゲインの初期値
    pid_gains[2] = 0.0f;                 // PIDゲインの初期値
    pid_gains_updated = false;           // PIDゲインの更新フラグ
}

void App::init()
{
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_SET);
    update_md_id();
    can_driver.set_board_id(md_id);
    can_driver.init();
    HAL_TIM_Base_Start_IT(&htim6);
    log_printf(LOG_INFO, "App initialized.\n");
}

void App::main_loop()
{
    if (initialized)
    {
        if (can_driver.get_target(&target))
        {
            update_target_count = 0;
            log_printf(LOG_DEBUG, "Target: %d\n", target);
        }
        else
        {
            update_target_count++;
        }

        if (update_target_count < update_target_count_max)
        {
            if (pid_gains[0] != 0.0f && target != 0)
            {
                output = motor_controller.calculate_pid(target, encoder) * (float)md_config.max_output;
            }
            else
            {
                motor_controller.reset_pid();
                output = target;
            }

            output = motor_controller.trapezoidal_control(output, md_config.max_acceleration);
        }
        else
        {
            output = 0;
            motor_controller.reset_pid();
        }

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
        motor_controller.run(output, md_config.max_output);
    }

    if (can_driver.get_init(&md_config))
    {
        log_printf(LOG_INFO, "md_canfig.max_output:%d\\n", md_config.max_output);
        log_printf(LOG_INFO, "md_config.max_acceleration:%d\n", md_config.max_acceleration);
        motor_controller.init(md_config.control_period);
        motor_controller.set_brake(true);
        motor_controller.get_count();
        motor_controller.reset_pid();
        HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
        initialized = true;
        log_printf(LOG_INFO, "MD initialized.\n");
    }

    if (can_driver.get_gain(0, pid_gains))
    {
        log_printf(LOG_INFO, "p_gain:%f\n", pid_gains[0]);
        can_driver.send_gain(0, pid_gains[0]);
        pid_gains_updated = true;
    }
    if (can_driver.get_gain(1, pid_gains + 1))
    {
        log_printf(LOG_INFO, "i_gain:%f\n", pid_gains[1]);
        can_driver.send_gain(1, pid_gains[1]);
    }
    if (can_driver.get_gain(2, pid_gains + 2))
    {
        log_printf(LOG_INFO, "d_gain:%f\n", pid_gains[2]);
        can_driver.send_gain(2, pid_gains[2]);
    }

    {
        motor_controller.set_pid_gain(pid_gains[0], pid_gains[1], pid_gains[2]);
        motor_controller.reset_pid();
    }

    if (limit_switch != HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin))
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin);
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
            can_driver.send_init(BOARD_KIND);
        }
        loop_count = 0;
    }
    else
    {
        loop_count++;
    }

    wait_for_next_period();
}

void App::timer_task()
{
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

void App::wait_for_next_period()
{
    while ((HAL_GetTick() - last_tick) < md_config.control_period)
    {
    }
    last_tick = HAL_GetTick();
}