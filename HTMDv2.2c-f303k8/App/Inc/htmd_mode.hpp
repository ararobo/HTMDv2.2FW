/**
 * @file htmd_mode.hpp
 * @author Gento Aiba
 * @brief MDのモードの共用体(通信時変換用)
 * @version 1.0
 * @date 2024-04-01
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

struct md_mode_flags
{
    unsigned char incremental_encoder : 1;
    unsigned char absolute_encoder : 1;
    unsigned char reverse_encoder : 1;
    unsigned char brake : 1;
    unsigned char pid : 1;
    unsigned char current : 1;
    unsigned char torque_control : 1;
    unsigned char state : 1;
};

union md_mode_t
{
    struct
    {
        struct md_mode_flags flags;
        struct
        {
            uint8_t max_acceleration;
            uint8_t max_current;
            uint8_t report_rate;
            uint16_t max_output;
            uint16_t motor_transfer_coefficient;
        } values;
    };
    uint8_t code[8];
};