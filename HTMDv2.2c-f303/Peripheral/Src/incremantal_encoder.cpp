/**
 * @file incremantal_encoder.cpp
 * @author  (8gn24gn25@gmail.com)
 * @brief インクリメンタルエンコーダのクラス
 * @version 1.0
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "incremental_encoder.hpp"
#include "tim.h"
#include <algorithm>

IncremantalEncoder::IncremantalEncoder()
{
}

void IncremantalEncoder::hardware_init()
{
    // エンコーダの初期化
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

int16_t IncremantalEncoder::get_count()
{
    // エンコーダのカウントを取得
    uint16_t enc_buff = TIM1->CNT;
    TIM1->CNT = 0;
    int16_t count = static_cast<int16_t>(enc_buff);
    return count;
}