/**
 * @file mcp3421.hpp
 * @author Gento Aiba (aiba-gento)
 * @brief MCP3421 18-bit ADC ドライバ（電流センシング用）
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
 * @brief MCP3421 I2C 18-bit ADC ドライバ
 *
 * 設定: 連続変換モード / 16-bit (15 SPS) / PGA ×1
 * 読み取り値の変換式:
 *   voltage_V = static_cast<int16_t>(raw) * 62.5e-6f
 */
class MCP3421
{
public:
    /**
     * @brief コンストラクタ
     *
     * @param hi2c 使用する I2C ハンドル
     */
    explicit MCP3421(I2C_HandleTypeDef& hi2c);

    /**
     * @brief 連続変換モードを開始する
     *
     */
    void init();

    /**
     * @brief ADC の変換結果を読み取る（符号付き: 逆電流対応）
     *
     * @param raw 生データを格納する変数のポインタ
     * @return true 新しい変換結果あり（RDY = 0）
     * @return false 変換中につき未更新（RDY = 1）
     */
    bool read(int16_t* raw);

private:
    I2C_HandleTypeDef& hi2c_;
    /// data(2) + config/RDY バイト(1) の計 3 バイト
    uint8_t buff_[3];

    static constexpr uint8_t ADDRESS = 0x68;
    /// 連続変換モード / 16-bit (15 SPS) / PGA ×1
    static constexpr uint8_t CONFIG      = 0b10011000;
    static constexpr uint32_t TIMEOUT_MS = 100;
};
