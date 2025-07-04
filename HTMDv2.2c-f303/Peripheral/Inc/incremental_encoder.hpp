/**
 * @file incremantal_encoder.hpp
 * @author  (8gn24gn25@gmail.com)
 * @brief インクリメンタルエンコーダのクラス
 * @version 1.0
 * @date 2025-05-16
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

class IncremantalEncoder
{
private:
    float enc_total;
    float max_count = 4096;

    float count_to_radian(int16_t count);

public:
    /**
     * @brief エンコーダのコンストラクタ
     */
    IncremantalEncoder();

    /**
     * @brief エンコーダの初期化
     *
     */
    void hardware_init();

    /**
     * @brief エンコーダのカウントを取得する
     *
     * @return int16_t エンコーダのカウント
     */
    int16_t get_count();

    /**
     * @brief エンコーダの値を角速度に変換する
     *
     * @param count エンコーダのカウント
     * @param period 制御周期[s]
     * @return float 角速度[rad/s]
     */
    float count_to_angular_velocity(int16_t count, float period);

    /**
     * @brief エンコーダの値をトータルで取得する
     *
     * @param encoder エンコーダのカウント
     * @return float トータルエンコーダの値[rad]
     */
    float total_encoder(int16_t encoder);

    void reset();
};