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
public:
    float enc_total;

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

    float total_encoder(uint16_t encoder);
};