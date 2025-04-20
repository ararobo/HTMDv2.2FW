#include "incremental_encoder.hpp"
#include "tim.h"
#include <algorithm>

IncremantalEncoder::IncremantalEncoder(uint16_t max_count)
{
    this->max_count = max_count;
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
    // カウントを制限
    std::clamp(count, int16_t(-max_count), int16_t(max_count));
    return count;
}