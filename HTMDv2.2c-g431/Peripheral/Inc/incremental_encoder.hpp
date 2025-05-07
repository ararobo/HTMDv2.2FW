/**
 * @file incremantal_encoder.hpp
 * @author  (8gn24gn25@gmail.com)
 * @brief インクリメンタルエンコーダのクラス
 * @version 0.1
 * @date 2025-05-06
 *
 * @copyright Copyright (c) 2025
 *
 */
#pragma once
#include <stdint.h>

class IncremantalEncoder
{
private:
    uint16_t max_count = 65535; // エンコーダの最大カウント
public:
    /**
     * @brief エンコーダのコンストラクタ
     *
     * @param max_count エンコーダの最大カウント
     */
    IncremantalEncoder(uint16_t max_count);

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
};