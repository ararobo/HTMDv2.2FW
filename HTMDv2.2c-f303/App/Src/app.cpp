#include "app.hpp"
#include "tim.h"
#include "motor_controller.hpp"
#include "serial_printf.hpp"
#define FW_VERSION 0x01
#define BOARD_KIND 0x01

CANDriver can_driver(0, BOARD_KIND, FW_VERSION);
MotorController motor_controller;

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
            if (pid_gain[0] != 0.0f && target != 0)
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

        if (loop_count > loop_count_max)
        {
            can_driver.send_limit_switch(limit_switch);
            HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
            loop_count = 0;
        }
        else
        {
            loop_count++;
        }
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

    if (can_driver.get_gain(0, pid_gain))
    {
        log_printf(LOG_INFO, "p_gain:%f\n", pid_gain[0]);
        can_driver.send_gain(0, pid_gain[0]);
    }
    if (can_driver.get_gain(1, pid_gain + 1))
    {
        log_printf(LOG_INFO, "i_gain:%f\n", pid_gain[1]);
        can_driver.send_gain(1, pid_gain[1]);
    }
    if (can_driver.get_gain(2, pid_gain + 2))
    {
        log_printf(LOG_INFO, "d_gain:%f\n", pid_gain[2]);
        can_driver.send_gain(2, pid_gain[2]);
    }

    {
        motor_controller.set_pid_gain(pid_gain[0], pid_gain[1], pid_gain[2]);
        motor_controller.reset_pid();
    }

    if (limit_switch != HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin))
    {
        limit_switch = HAL_GPIO_ReadPin(LIM1_GPIO_Port, LIM1_Pin);
        can_driver.send_limit_switch(limit_switch);
    }

    HAL_Delay(md_config.control_period);
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