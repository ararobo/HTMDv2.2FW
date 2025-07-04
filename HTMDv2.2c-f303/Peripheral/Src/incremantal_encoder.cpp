/**
 * @file incremantal_encoder.cpp
 * @author aiba-gento, Watanabe-Koichiro
 * @brief インクリメンタルエンコーダのクラス
 * @version 2.0
 * @date 2025-07-05
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "incremental_encoder.hpp"
#include "tim.h"
#define M_PI 3.14159265358979323846f

IncremantalEncoder::IncremantalEncoder()
{
}

float IncremantalEncoder::count_to_radian(int16_t count)
{
    // エンコーダのカウントをラジアンに変換
    float radian = float(count) / max_count * 2.0f * M_PI; // 2πで正規化
    return radian;
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

float IncremantalEncoder::count_to_angular_velocity(int16_t count, float period)
{
    // エンコーダの値を角速度に変換
    float angular_velocity = count_to_radian(count) / period; // ラジアン/秒
    return angular_velocity;
}

float IncremantalEncoder::total_encoder(int16_t encoder)
{
    // エンコーダの値をトータルで取得
    enc_total += count_to_radian(encoder);
    return enc_total;
}

void IncremantalEncoder::reset()
{
    // エンコーダのトータル値をリセット
    enc_total = 0.0f;
    TIM1->CNT = 0; // カウントもリセット
}