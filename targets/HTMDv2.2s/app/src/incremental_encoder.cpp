/**
 * @file incremental_encoder.cpp
 * @author Gento Aiba (aiba-gento), Watanabe-Koichiro
 * @brief インクリメンタルエンコーダの具象クラス
 * @version 3.0
 * @date 2026-02-23
 *
 * @copyright Copyright (c) 2025
 *
 */
#include "app/incremental_encoder.hpp"

#include <cmath>
#include <cstdint>

#include "tim.h"

// 2π 定数 (M_PI は POSIX 拡張のため constexpr で定義)
static constexpr float TWO_PI = 6.28318530f;

IncrementalEncoder::IncrementalEncoder(uint16_t max_count) : max_count_(max_count), enc_total_(0.0f)
{
}

void IncrementalEncoder::hardware_init()
{
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
}

int16_t IncrementalEncoder::read_and_reset_count()
{
    // 読み取りとリセットの間に割り込みが入ると最大 1 ティックの誤差が生じる
    // 許容誤差範囲内であるため対策しない
    uint16_t raw = TIM1->CNT;
    TIM1->CNT    = 0U;
    return static_cast<int16_t>(raw);
}

float IncrementalEncoder::count_to_rad(int16_t count) const
{
    return static_cast<float>(count) / static_cast<float>(max_count_) * TWO_PI;
}

float IncrementalEncoder::count_to_angular_velocity(int16_t count, float period_s)
{
    return count_to_rad(count) / period_s;
}

float IncrementalEncoder::accumulate_angle_rad(int16_t count)
{
    enc_total_ += count_to_rad(count);
    return enc_total_;
}

void IncrementalEncoder::reset()
{
    enc_total_ = 0.0f;
    TIM1->CNT  = 0U;
}