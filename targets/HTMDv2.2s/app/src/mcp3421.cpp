/**
 * @file mcp3421.cpp
 * @author Gento Aiba (aiba-gento)
 * @brief MCP3421 18-bit ADC ドライバ（電流センシング用）
 * @version 0.2.0
 * @date 2026-03-08
 *
 * @copyright Copyright (c) 2026 ararobo
 * SPDX-License-Identifier: Apache-2.0
 *
 */
#include "app/mcp3421.hpp"

MCP3421::MCP3421(I2C_HandleTypeDef& hi2c) : hi2c_(hi2c) {}

/**
 * @brief 連続変換モードを開始する
 *
 */
void MCP3421::init()
{
    uint8_t config = CONFIG;
    HAL_I2C_Master_Transmit(&hi2c_, ADDRESS << 1, &config, 1, TIMEOUT_MS);
}

/**
 * @brief ADC の変換結果を読み取る
 *
 * @param raw 生データを格納する変数のポインタ
 *            変換式: voltage_V = static_cast<int16_t>(raw) * 62.5e-6f
 * @return true 新しい変換結果あり（RDY = 0）
 * @return false 変換中につき未更新（RDY = 1）
 */
bool MCP3421::read(int16_t* raw)
{
    // 16-bit モード: data(2) + config/RDY バイト(1) の計 3 バイトを読む
    if (HAL_I2C_Master_Receive(&hi2c_, ADDRESS << 1, buff_, 3, TIMEOUT_MS) != HAL_OK) {
        return false;
    }

    // RDY bit (bit7 of config byte) == 1 は変換中を示す
    if (buff_[2] & 0x80) {
        return false;
    }

    *raw = static_cast<int16_t>((buff_[0] << 8) | buff_[1]);
    return true;
}
