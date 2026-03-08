/**
 * @file tmp275.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief TMP275 温度センサードライバ
 * @version 0.2.0
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#pragma once
#include <stdint.h>

#include "i2c.h"

/**
 * @brief TMP275 I2C 温度センサードライバ
 *
 * 読み取り値の変換式:
 *   temperature_degC = static_cast<int16_t>(raw) / 256.0f
 */
class TMP275
{
public:
    /**
     * @brief コンストラクタ
     *
     * @param hi2c 使用する I2C ハンドル
     */
    explicit TMP275(I2C_HandleTypeDef& hi2c);

    /**
     * @brief ポインタレジスタを温度レジスタ (0x00) に設定する
     *
     */
    void init();

    /**
     * @brief 温度の生データを読み取る（符号付き: 負温度対応）
     *
     * @param raw 生データを格納する変数のポインタ
     * @return true 正常に読み取れた
     * @return false 読み取り失敗
     */
    bool read(int16_t* raw);

private:
    I2C_HandleTypeDef& hi2c_;
    uint8_t buff_[2];

    static constexpr uint8_t ADDRESS     = 0x48;
    static constexpr uint32_t TIMEOUT_MS = 100;
};
